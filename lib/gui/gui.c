/**
 * @file gui.c
 * @brief LVGL graphical user interface creation and control.
 *
 * This file contains functions to initialize and manage the various
 * screens (Boot, Main, ECG), status bars, and the sidebar menu using
 * the LVGL graphics library. It also handles encoder input mappings
 * and GUI state transitions.
 */

#include "../gui/gui.h"
#include "../lcd/lcd.h"


#include "encoder.h"
#include "esp_log.h"

#define SIDEBAR_W 80
#define DATA_BAR_H 20
#define STATUS_BAR_H 20

#define WAVEFORM_CHART_NUM_POINTS 125 // (for decim=128) //500 (for decim=32)

#define _TESTING 1

LV_IMG_DECLARE(SetupImg);

typedef enum {
    OFFSET = 0,
    SCALE = 1
} chart_tool_t;

lv_obj_t *data_bar = NULL;
lv_obj_t *class_label = NULL;
lv_obj_t *conf_label = NULL;

lv_obj_t *status_bar = NULL;
lv_obj_t *status_label = NULL;
lv_obj_t *tool_label = NULL;

lv_obj_t *scr_container = NULL;
lv_obj_t *ecg_scr_label;

static lv_waveform_t *waveform_ptr = NULL;

static lv_obj_t *root_scr = NULL;
static lv_obj_t *sidebar = NULL;
static rotary_encoder_event_t enc_event;
static lv_indev_t *enc_dev = NULL;
static lv_group_t *sidebar_group = NULL;
static lv_style_t def_item_style;
static lv_style_t hover_item_style;
static bool styles_initialized = false;

static lv_obj_t *boot_bar = NULL;
static lv_obj_t *boot_scr = NULL;
lv_obj_t *main_scr = NULL;
lv_obj_t *ecg_scr = NULL;

static bool sidebar_shown = true;
static chart_tool_t chart_tool_state = OFFSET;      // = 0

static const char *TAG = "gui.c";

/**
 * @brief Creates the base/root screen for the LVGL UI.
 */
void create_root_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In create_root_screen()");
    root_scr = lv_obj_create(NULL);

    lv_obj_remove_flag(root_scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(root_scr, lv_color_hex(0x121212), 0);
    lv_obj_set_style_bg_opa(root_scr, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(root_scr, 0, 0);

    // lv_screen_load(root_scr);

    if (_TESTING) ESP_LOGI(TAG, "Returning from create_root_screen()");
}

/**
 * @brief Retrieves the root screen which houses the main containers.
 * 
 * @return lv_obj_t* Pointer to the root screen object.
 */
lv_obj_t *get_root_screen(void) {
    if (!root_scr) {
        create_root_screen();
    }
    return root_scr;
}

/**
 * @brief Updates the system state text displayed in the status bar.
 * 
 * @param new_state_data Pointer to the string representing the new state.
 */
void update_sys_state_text(void *new_state_data) {
    char *new_state_text = (char *)new_state_data;

    if (status_label) {
        // Safely update the text from within the LVGL task context
        lv_label_set_text_fmt(status_label, "Status: %s", new_state_text);
        if (_TESTING) ESP_LOGI(TAG, "Status bar (state) updated to: %s", new_state_text);
    } else {
        if (_TESTING) ESP_LOGW(TAG, "update_sys_state_text(%s) was called but status_label is not defined.", new_state_text);
    }
}

/**
 * @brief Updates the tool text displayed in the status bar.
 * 
 * @param new_tool_text String containing the new tool state (e.g., Scale, Offset).
 */
void update_tool_text(const char *new_tool_text) {
    ESP_LOGI(TAG, "In update_tool_text()");
    if (tool_label) {
        lv_label_set_text_fmt(tool_label, "Tool: %s", new_tool_text);
        if (_TESTING) ESP_LOGI(TAG, "Status bar (tool) updated.");
    } else {
        if (_TESTING) ESP_LOGW(TAG, "update_sys_state_text(%s) was called but tool_label is not defined.", new_tool_text);
    }
}

/**
 * @brief Creates the bottom status bar containing state and tool labels.
 */
static void create_status_bar() {
    if (_TESTING) ESP_LOGI(TAG, "In create_status_bar()");
    if (!root_scr) {
        create_root_screen();
    }

    status_bar = lv_obj_create(root_scr);
    lv_obj_set_size(status_bar, LV_HOR_RES - SIDEBAR_W, STATUS_BAR_H);
    lv_obj_set_layout(status_bar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(status_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(status_bar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_align(status_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_pad_all(status_bar, 0, 0);
    lv_obj_set_style_border_width(status_bar, 0, 0);
    lv_obj_set_style_bg_opa(status_bar, LV_OPA_COVER, 0);

    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(status_bar, 0, 0);

    // Create two labels inside data_bar
    status_label = lv_label_create(status_bar);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_label_set_text_fmt(status_label, "Status: Ready");
    // lv_label_set_text_fmt(status_label, "Status: --   ");

    tool_label = lv_label_create(status_bar);
    lv_obj_set_style_text_color(tool_label, lv_color_white(), 0);
    lv_label_set_text_fmt(tool_label, "Tool: --  ");

    if (_TESTING) ESP_LOGI(TAG, "Returning from create_status_bar()");
}

/**
 * @brief Updates the classification and confidence labels in the data bar.
 * 
 * @param new_classification_text String representing the ML class prediction.
 * @param new_confidence_level Float representing the confidence probability (0.0 to 1.0).
 */
void update_data_bar_text(const char *new_classification_text, float new_confidence_level) {
    ESP_LOGI(TAG, "In update_data_bar_text()");
    // Define the text buffer
    char percent_confidence[25];
    // Stuff the char buffer
    snprintf(percent_confidence, sizeof(percent_confidence), "%.0f%%", 100 * new_confidence_level);     // remove decimals from float
    if (class_label && conf_label) {
        lv_label_set_text_fmt(class_label, "Classification: %s", new_classification_text);
        lv_label_set_text_fmt(conf_label, "Confidence: %s", percent_confidence);
        ESP_LOGI(TAG, "Data bar updated.");
    } else {
        if (_TESTING) ESP_LOGW(TAG, "update_data_bar_text(%s, %f) was called but data_bar_label is not defined.", new_classification_text, new_confidence_level);
    }
}

/**
 * @brief Creates the top data bar displaying ML prediction results.
 */
static void create_data_bar() {
    if (_TESTING) ESP_LOGI(TAG, "In create_data_bar()");
    if (!root_scr) {
        create_root_screen();
    }

    data_bar = lv_obj_create(root_scr);
    lv_obj_set_size(data_bar, LV_HOR_RES - SIDEBAR_W, DATA_BAR_H);
    lv_obj_set_layout(data_bar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(data_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(data_bar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    lv_obj_align(data_bar, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_pad_all(data_bar, 0, 0);
    lv_obj_set_style_border_width(data_bar, 0, 0);
    lv_obj_set_style_bg_opa(data_bar, LV_OPA_COVER, 0);

    lv_obj_set_style_bg_color(data_bar, lv_color_hex(0x252525), 0);
    lv_obj_set_style_radius(data_bar, 0, 0);

    // Create two labels inside data_bar
    class_label = lv_label_create(data_bar);
    lv_obj_set_style_text_color(class_label, lv_color_white(), 0);
    lv_label_set_text_fmt(class_label, "Classification: --");
    
    conf_label = lv_label_create(data_bar);
    lv_obj_set_style_text_color(conf_label, lv_color_white(), 0);
    lv_label_set_text_fmt(conf_label, "Confidence: --  ");

    if (_TESTING) ESP_LOGI(TAG, "Returning from create_data_bar()");
}

/**
 * @brief Creates the main central container for holding sub-screens.
 */
static void create_scr_container(void) {
    if (!scr_container) {
        if (!root_scr) {
            create_root_screen();
        }
        scr_container = lv_obj_create(root_scr);
        lv_obj_align(scr_container, LV_ALIGN_TOP_LEFT, 0, DATA_BAR_H);      // offset down by the height of the data bar
        lv_obj_set_size(scr_container, LV_HOR_RES - SIDEBAR_W, LV_VER_RES - DATA_BAR_H - STATUS_BAR_H);
        lv_obj_set_style_pad_all(scr_container, 0, 0);
        lv_obj_set_style_border_width(scr_container, 0, 0);
        lv_obj_set_style_bg_opa(scr_container, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(scr_container, 0, 0);

        lv_obj_remove_flag(scr_container, LV_OBJ_FLAG_SCROLLABLE);
    }
}

/**
 * @brief Callback function for the encoder-based LVGL input device.
 * 
 * @note Called continuously by LVGL to poll the encoder state.
 * 
 * @param indev Pointer to the LVGL input device.
 * @param data Pointer to the input device data structure to populate.
 */
static void enc_read(lv_indev_t *indev, lv_indev_data_t *data) {
    if (xQueueReceive(forwarded_enc_queue, &enc_event, 0) != pdFALSE) {
        // Reset display timeout due to recorded user input
        reset_display_timeout();

        // Determine appropriate system functionality based on input event
        if (enc_event.type != RE_ET_CHANGED) {
            data->enc_diff = 0;
        } else {
            data->enc_diff = enc_event.diff;    // modify encoder value in lv_indev
            
            // Only update chart axes if on ECG screen and if sidebar is not shown
            if (!lv_obj_has_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN) && !sidebar_shown) {
                if (chart_tool_state) {
                    update_chart_scale(waveform_ptr, enc_event.diff);
                } else {
                    update_chart_offset(waveform_ptr, enc_event.diff);
                }
                if (_TESTING) ESP_LOGI(TAG, "Enc Read sees encoder value changed");
            }
        }
        if (enc_event.type == RE_ET_BTN_LONG_PRESSED) {
            // Only update chart axes if on ECG screen
            if (!lv_obj_has_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN)) {
                chart_tool_state = !chart_tool_state;
                if (chart_tool_state) {   // 
                    update_tool_text("Scale ");     // extra space to keep padding consistent
                } else {
                    update_tool_text("Offset");
                }

                if (_TESTING) ESP_LOGI(TAG, "Enc Read sees long button press");
            }
        }
        
        if (enc_event.type == RE_ET_BTN_CLICKED) {      // button pressed
            data->state = LV_INDEV_STATE_PRESSED;   // modify lv_indev state
            if (_TESTING) ESP_LOGI(TAG, "Enc Read sees button pressed");
        } else {    //  button released
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {    // no change
        data->enc_diff = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/**
 * @brief Initializes the encoder as an LVGL input device and creates the UI group.
 * @note Called from create_sidebar().
 */
static void init_sidebar_input(void) {
    if (!enc_dev) {
        enc_dev = lv_indev_create();
        lv_indev_set_type(enc_dev, LV_INDEV_TYPE_ENCODER);
        lv_indev_set_read_cb(enc_dev, enc_read);
    }

    if (!sidebar_group) {
        sidebar_group = lv_group_create();
    }

    lv_indev_set_group(enc_dev, sidebar_group);
}

/**
 * @brief Initializes default and focused item styles for the sidebar.
 * 
 * @note Sets the styles_initialized flag to true to prevent redundant calls.
 */
static void init_item_styles(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_item_styles()");
    if (styles_initialized) return;

    lv_style_init(&def_item_style);
    lv_style_set_bg_color(&def_item_style, lv_color_hex(0x333333));
    lv_style_set_bg_opa(&def_item_style, LV_OPA_COVER);
    lv_style_set_text_color(&def_item_style, lv_color_white());
    lv_style_set_border_width(&def_item_style, 0);

    lv_style_init(&hover_item_style);
    lv_style_set_bg_color(&hover_item_style, lv_color_hex(0xdceaeb));
    lv_style_set_bg_opa(&hover_item_style, LV_OPA_COVER);
    lv_style_set_text_color(&hover_item_style, lv_color_black());

    styles_initialized = true;
}

/**
 * @brief Toggles the visibility of the sidebar and resizes main containers.
 * 
 * @param show_sidebar True to show the sidebar, false to hide it.
 */
static void update_sidebar_state(bool show_sidebar) {
    // Update main container size
    if (show_sidebar) {
        lv_obj_set_size(scr_container, LV_HOR_RES - SIDEBAR_W, LV_VER_RES - DATA_BAR_H - STATUS_BAR_H);
        lv_obj_set_size(data_bar, LV_HOR_RES - SIDEBAR_W, DATA_BAR_H);
        lv_obj_set_size(status_bar, LV_HOR_RES - SIDEBAR_W, STATUS_BAR_H);

    } else {
        lv_obj_set_size(scr_container, LV_HOR_RES, LV_VER_RES - DATA_BAR_H - STATUS_BAR_H);
        lv_obj_set_size(data_bar, LV_HOR_RES, DATA_BAR_H);
        lv_obj_set_size(status_bar, LV_HOR_RES, STATUS_BAR_H);
    }

    lv_obj_set_flag(sidebar, LV_OBJ_FLAG_HIDDEN, !show_sidebar);
    sidebar_shown = show_sidebar;
}

/**
 * @brief LVGL event callback for sidebar item clicks.
 * 
 * @details Calls load_system_state() to update the GUI task with the desired state.
 *          If the sidebar is hidden when clicked, it will show the sidebar instead.
 * 
 * @param e Pointer to the LVGL event.
 */
static void sidebar_item_event_cb(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (sidebar_shown) {
        system_state_t state = (system_state_t)lv_event_get_user_data(e);
    
        if (_TESTING) ESP_LOGI(TAG, "sidebar_item_event_cb() got passed a LV_EVENT_CLICKED: %d", (state));
    
        load_system_state(state);

        update_sidebar_state(false);
    } else {
        update_sidebar_state(true);
    }
}

/**
 * @brief Reusable function for creating a sidebar item with button-like styling.
 * 
 * @param label_text The text displayed on the button.
 * @param gui_state The target system state to transition to when clicked.
 */
void create_sidebar_item(const char *label_text, system_state_t gui_state) {
    if (_TESTING) ESP_LOGI(TAG, "In create_sidebar_item()");

    // Create the button item inside the sidebar
    lv_obj_t *item = lv_obj_create(sidebar);

    // Width fills most of sidebar (100% minus sidebar padding), height grows to fill vertical space
    lv_obj_set_width(item, lv_pct(100));
    lv_obj_set_flex_grow(item, 1);

    // Inner padding so label isn't glued to edges
    lv_obj_set_style_pad_all(item, 5, 0);

    // Margin between items (vertical spacing)
    lv_obj_set_style_margin_bottom(item, 5, 0);

    // Borders
    lv_obj_set_style_border_width(item, 0, 0);      // no border
    // lv_obj_set_style_border_color(item, lv_color_white(), 0);
    // lv_obj_set_style_border_opa(item, LV_OPA_COVER, 0);

    // Create default and focused/hover styles for background and text
    init_item_styles();

    // Label inside the button, centered horizontally and vertically
    lv_obj_t *item_label = lv_label_create(item);
    lv_label_set_text(item_label, label_text);
    lv_obj_center(item_label);

    // Styles for default and focused states
    lv_obj_add_style(item, &def_item_style, 0);
    lv_obj_add_style(item, &hover_item_style, LV_STATE_FOCUSED);

    // Make it clickable
    lv_obj_add_flag(item, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);

    // Event callback
    // This sidebar item is now registered to the system_state_t passed to create_sidebar_item
    // When LV_EVENT_CLICKED is triggered, this state will be passed to load_system_state() through sidebar_item_event_cb()
    lv_obj_add_event_cb(item, sidebar_item_event_cb, LV_EVENT_CLICKED, (void *)gui_state);

    // Add item to the encoder group
    lv_group_add_obj(sidebar_group, item);
}


/**
 * @brief Creates LVGL objects that function as the main menu sidebar.
 */
void create_sidebar(void) {
    if (_TESTING) ESP_LOGI(TAG, "In create_sidebar()");

    if (!root_scr) {
        create_root_screen();
    }

    // Create the sidebar container
    sidebar = lv_obj_create(root_scr);

    // Size & alignment
    lv_obj_set_size(sidebar, SIDEBAR_W, LV_VER_RES);
    lv_obj_align(sidebar, LV_ALIGN_TOP_RIGHT, 0, 0);

    // Flex layout for vertical stacking of buttons
    lv_obj_set_layout(sidebar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(sidebar, LV_FLEX_FLOW_COLUMN);

    // Horizontal alignment: center the items inside the sidebar
    lv_obj_set_flex_align(sidebar, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, 0);

    lv_obj_remove_flag(sidebar, LV_OBJ_FLAG_SCROLLABLE);

    // Padding inside the sidebar (space between edge and items)
    lv_obj_set_style_pad_top(sidebar, 10, 0);
    lv_obj_set_style_pad_bottom(sidebar, 10, 0);
    lv_obj_set_style_pad_left(sidebar, 5, 0);
    lv_obj_set_style_pad_right(sidebar, 5, 0);

    // Sidebar background color
    lv_obj_set_style_bg_color(sidebar, lv_color_hex(0x252525), 0);
    lv_obj_set_style_bg_opa(sidebar, LV_OPA_COVER, 0);

    lv_obj_set_style_border_width(sidebar, 0, 0);   // no border

    lv_obj_set_style_radius(sidebar, 0, 0);

    // Initialize encoder input for sidebar
    init_sidebar_input();

    // Remove all previous objects in sidebar group
    lv_group_remove_all_objs(sidebar_group);

    // Create buttons
    create_sidebar_item("HOME", GUI_MAIN);
    create_sidebar_item("ECG", GUI_ECG);

    // Set initial focus on first sidebar item
    lv_obj_t *focus_item = lv_obj_get_child(sidebar, 0);
    if (focus_item) {
        lv_group_focus_obj(focus_item);
    }

    // Make the sidebar able to be hidden
    lv_obj_add_flag(sidebar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_flag(sidebar, LV_OBJ_FLAG_HIDDEN, false);
}

/**
 * @brief Helper function to set the current tool text based on chart state.
 * 
 * @param is_ECG_shown True if the ECG screen is currently active.
 */
static void set_tool_text(bool is_ECG_shown) {
    if (!is_ECG_shown) {
        update_tool_text("--    ");     // extra space to keep padding consistent
        return;
    }
    if (chart_tool_state) {   // 1 = scale
        update_tool_text("Scale ");     // extra space to keep padding consistent
    } else {                    // 0 = offset
        update_tool_text("Offset");
    }
}

/**
 * @brief Initializes the boot screen UI objects and progress bar.
 */
static void create_boot_screen() {
    if (_TESTING) ESP_LOGI(TAG, "In create_boot_screen()");   
    
    boot_scr = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(boot_scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(boot_scr, LV_OPA_COVER, 0);
    lv_obj_align(boot_scr, LV_ALIGN_LEFT_MID, 0, 0);
    
    lv_obj_t *text = lv_label_create(boot_scr);
    lv_label_set_text(text, "Booting PulsePioneer...");
    lv_obj_set_style_text_color(text, lv_color_white(), 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, -40);
    
    boot_bar = lv_bar_create(boot_scr);
    lv_obj_set_size(boot_bar, 240, 30);
    lv_obj_align(boot_bar, LV_ALIGN_CENTER, 0, 40);
}

/**
 * @brief Callback to exit the system from boot state and enter the nominal use state.
 */
static void boot_bar_completed_cb() {
    // Replace boot screen with main root screen container
    lv_screen_load(root_scr);
    // Switch to main menu GUI state
    load_system_state(GUI_MAIN);
}

/**
 * @brief Increments the value of the boot bar and triggers completion callback.
 */
static void start_boot_bar_animation() {
    if (_TESTING) ESP_LOGI(TAG, "In show_boot_bar_animation()");

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, boot_bar);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 1000);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_bar_set_value);
    lv_anim_set_completed_cb(&a, (lv_anim_completed_cb_t)boot_bar_completed_cb);
    lv_anim_start(&a);
}

/**
 * @brief Turns on LCD backlight, loads the boot screen, and starts animation.
 */
void show_boot_screen() {
    if (_TESTING) ESP_LOGI(TAG, "In show_boot_screen()");   
    
    // Initialize LVGL objects for boot screen
    create_boot_screen();
    // Show boot screen
    lv_screen_load(boot_scr);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Turn on LCD backlight
    set_led_pwm(100);
    // Activate boot bar animation
    // Enters main screen upon completion
    start_boot_bar_animation();
}

// ------------------------------
// Main Screen
// ------------------------------
static void create_main_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In create_main_screen()");

    main_scr = lv_obj_create(scr_container);
    
    if (!main_scr) {
        if (_TESTING) ESP_LOGI(TAG, "main_scr is still null");
    }
    
    lv_obj_set_style_bg_color(main_scr, lv_color_hex(0xd4f8fc), 0);
    lv_obj_set_style_bg_opa(main_scr, LV_OPA_COVER, 0);
    lv_obj_set_size(main_scr, lv_pct(100), lv_pct(100));
    lv_obj_align(main_scr, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(main_scr, 0, 0);
    
    // Disable scrolling on this screen brought about by adding SetupImg
    lv_obj_clear_flag(main_scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flag(main_scr, LV_OBJ_FLAG_SCROLLABLE, false);    

    lv_obj_t * img = lv_img_create(main_scr);
    // Set the image source to the C array
    lv_img_set_src(img, &SetupImg);
    // Position it in the center
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    // Down-scale the image by zooming out
    lv_img_set_zoom(img, 128);

    // lv_obj_t *text = lv_label_create(main_scr);
    // lv_label_set_text(text, "Welcome to Pulse Pioneer");    // TODO: add setup instructions
    // lv_obj_set_style_text_color(text, lv_color_black(), 0);
    // lv_obj_set_style_text_align(text, LV_TEXT_ALIGN_CENTER, 0);
    // lv_obj_align(text, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_flag(main_scr, LV_OBJ_FLAG_HIDDEN);

    if(_TESTING) ESP_LOGI(TAG, "Returning from create_main_screen()");
}

/**
 * @brief Initializes the LVGL objects for the main/home screen.
 */
void show_main_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In show_main_menu()");

    // Create screen if not previously created
    if (!main_scr) {
        create_main_screen();
    }

    // Show the screen
    lv_obj_set_flag(main_scr, LV_OBJ_FLAG_HIDDEN, false);

    // No tool defined on home screen
    set_tool_text(false);

    if(_TESTING) ESP_LOGI(TAG, "Returning from show_main_scr()");
}


// ------------------------------
// ECG Screen
// ------------------------------
/*
Use to pass the created waveform pointer to other functions for external use.
*/
lv_waveform_t *get_waveform_ptr() {
    return waveform_ptr;
}

/**
 * @brief Adds data series channels to the ECG waveform chart.
 */
static void add_waveform_plot() {
    if(_TESTING) ESP_LOGI(TAG, "In add_waveform_plot()");
    if (!waveform_ptr) {
        ESP_LOGE(TAG, "Waveform_ptr has not be initialized. Call create_ECG_screen() prior to add_waveform_plot().");
        return;
    }

    if (!waveform_ptr->ch1) {
        waveform_ptr->ch1 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    }
    if (!waveform_ptr->ch2) {
        waveform_ptr->ch2 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    }
    if (!waveform_ptr->ch3) {
        waveform_ptr->ch3 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_LIGHT_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    }
}

/**
 * @brief Retrieves the ECG screen calibration status label.
 * 
 * @return lv_obj_t* Pointer to the LVGL label object.
 */
lv_obj_t *get_ecg_scr_label() {
    return ecg_scr_label;
}

/**
 * @brief Initializes the boot screen UI objects and progress bar.
 */
static void create_ECG_screen(uint8_t num_charts) {
    if (_TESTING) ESP_LOGI(TAG, "In create_ECG_screen()");
    static lv_waveform_t waveform = {};
    waveform.chart = NULL;
    waveform.ch1 = NULL;
    waveform.ch2 = NULL;
    waveform.ch3 = NULL;
    waveform.y_scale = NULL;

    ecg_scr = lv_obj_create(scr_container);
    lv_obj_set_size(ecg_scr, lv_pct(100), lv_pct(100));
    lv_obj_align(ecg_scr, LV_ALIGN_CENTER, 0, 0);
    lv_obj_remove_flag(ecg_scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(ecg_scr, 0, 0);

    ecg_scr_label = lv_label_create(ecg_scr);
    lv_label_set_text(ecg_scr_label, "Calibrating ECG. One moment please...");
    lv_obj_set_style_text_color(ecg_scr_label, lv_color_black(), 0);
    lv_obj_align(ecg_scr_label, LV_ALIGN_CENTER, 0, 0);

    // Initialize chart object
    waveform.chart = lv_chart_create(ecg_scr);

    // Set size to be 100% of both height and width of parent container
    lv_obj_set_size(waveform.chart, lv_pct(100), lv_pct(100));
    lv_obj_center(waveform.chart);
    lv_chart_set_type(waveform.chart, LV_CHART_TYPE_LINE);
    // Set the number of data points shown on the chart at once
    // Effectively sets the data resolution of the chart
    lv_chart_set_point_count(waveform.chart, WAVEFORM_CHART_NUM_POINTS);
    // Make individual points invisible
    lv_obj_set_style_height(waveform.chart, 0, LV_PART_INDICATOR);
    lv_obj_set_style_width(waveform.chart, 0, LV_PART_INDICATOR);
    lv_obj_set_style_line_width(waveform.chart, 3, LV_PART_ITEMS);
    
    // Set update mode to CIRCULAR to prevent full-screen redraws
    // Reduces the rendering and SPI DMA delay from ~100ms down to ~2ms
    lv_chart_set_update_mode(waveform.chart, LV_CHART_UPDATE_MODE_CIRCULAR);
    
    // Pack the constructed waveform into the global pointer
    waveform_ptr = &waveform;

    create_chart_scale(waveform_ptr);

    if (num_charts > 0 && num_charts < 4) {
        while (num_charts-- != 0) {
            add_waveform_plot();
        }
    } else {
        if (_TESTING) ESP_LOGE(TAG, "Parameter num_charts is out of bounds.");
        return;
    }

    lv_obj_add_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN);
}

/**
 * @brief Displays the main/home screen and updates the tool status.
 */
void show_ECG_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In show_ECG_screen()");
    if (!ecg_scr) {
        create_ECG_screen(1);
    }
    // Show the ECG screen
    lv_obj_set_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN, false);

    set_tool_text(true);

    // Resume the ECG data streaming task
    vTaskResume(ecg_stream_task_handle);
    if (_TESTING) ESP_LOGI(TAG, "ecg_stream_task() resumed from show_ECG_screen().");
}

/**
 * @brief Orchestrates the initialization of all main GUI screens using LVGL.
 */
void create_LVGL_screens() {
    if (_TESTING) ESP_LOGI(TAG, "In create_LVGL_screens()");
    // Make system data bar
    create_data_bar();
    // Make system status bar
    create_status_bar();
    // Make container for GUI subscreens
    create_scr_container();
    // Initialize LVGL objects for main screen and hide the screen
    create_main_screen();
    // Initialize LVGL objects for ECG screen with only one chart object and hide the screen
    create_ECG_screen(1);
    // Initialize LVGL objects for system menu sidebar
    create_sidebar();
}
