/*
Functions related to creating and controlling the LVGL graphical user interface.
*/

#include "gui.h"

#define SIDEBAR_W 80
#define STATUS_BAR_H 20

#define _TESTING 1

lv_obj_t *status_bar = NULL;
lv_obj_t *status_bar_label = NULL;

lv_obj_t *scr_container = NULL;
lv_obj_t *ecg_scr_label;

static lv_waveform_t *waveform_ptr = NULL;
static TaskHandle_t ecg_stream_task_handle = NULL;

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

const char *TAG = "gui.c";

void pass_ecg_stream_task_handle(TaskHandle_t *handle) {
    ecg_stream_task_handle = *handle;
}

TaskHandle_t get_ecg_stream_task_handle() {
    return ecg_stream_task_handle;
}

void create_root_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In create_root_screen()");
    root_scr = lv_obj_create(NULL);

    lv_obj_remove_flag(root_scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(root_scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(root_scr, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(root_scr, 0, 0);

    // lv_screen_load(root_scr);

    if (_TESTING) ESP_LOGI(TAG, "Returning from create_root_screen()");
}

/*
Returns the root screen which houses the screen container and sidebar container.
*/
lv_obj_t *get_root_screen(void) {
    if (!root_scr) {
        create_root_screen();
    }
    return root_scr;
}

void update_sys_state_text(const char *new_state_text) {
    if (status_bar_label) {
        lv_label_set_text_fmt(status_bar_label, "Status: %s", new_state_text);
    } else {
        if (_TESTING) ESP_LOGW(TAG, "update_sys_state_text(%s) was called but status_bar_label is not defined.", new_state_text);
    }
}

static void create_status_bar() {
    if (_TESTING) ESP_LOGI(TAG, "In create_status_bar()");
    if (!root_scr) {
        create_root_screen();
    }

    status_bar = lv_obj_create(root_scr);
    lv_obj_set_size(status_bar, LV_HOR_RES, STATUS_BAR_H);
    lv_obj_align(status_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_pad_all(status_bar, 0, 0);
    lv_obj_set_style_border_width(status_bar, 0, 0);
    lv_obj_set_style_bg_opa(status_bar, LV_OPA_COVER, 0);

    lv_obj_set_style_bg_color(status_bar, lv_color_make(50, 50, 50), 0);
    lv_obj_set_style_radius(status_bar, 0, 0);

    status_bar_label = lv_label_create(status_bar);
    lv_obj_align(status_bar_label, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_text_color(status_bar_label, lv_color_white(), 0);
    lv_label_set_text_fmt(status_bar_label, "Status: none");

    if (_TESTING) ESP_LOGI(TAG, "Returning from create_status_bar()");
}

static void create_scr_container(void) {
    if (!scr_container) {
        if (!root_scr) {
            create_root_screen();
        }
        scr_container = lv_obj_create(root_scr);
        lv_obj_align(scr_container, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_size(scr_container, LV_HOR_RES - SIDEBAR_W, LV_VER_RES - STATUS_BAR_H);
        lv_obj_set_style_pad_all(scr_container, 0, 0);
        lv_obj_set_style_border_width(scr_container, 0, 0);
        lv_obj_set_style_bg_opa(scr_container, LV_OPA_TRANSP, 0);
        lv_obj_set_style_radius(scr_container, 0, 0);

        lv_obj_remove_flag(scr_container, LV_OBJ_FLAG_SCROLLABLE);
    }
}

/*
Callback function for an encoder-based LVGL input device.
Called from init_sidebar_input().
*/
static void enc_read(lv_indev_t *indev, lv_indev_data_t *data) {
    if (xQueueReceive(forwarded_enc_queue, &enc_event, 0) != pdFALSE) {
        reset_display_timeout();
        if (enc_event.type != RE_ET_CHANGED) {
            data->enc_diff = 0;
        } else {
            data->enc_diff = enc_event.diff;
            if (_TESTING) ESP_LOGI(TAG, "Enc Read sees encoder value changed");
        }
        
        if (enc_event.type == RE_ET_BTN_CLICKED) {
            data->state = LV_INDEV_STATE_PRESSED;
            if (_TESTING) ESP_LOGI(TAG, "Enc Read sees button pressed");
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->enc_diff = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/*
Called from create_sidebar() to initialize the encoder as an LVGL input device and create the LVGL group.
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

/*
Called from create_sidebar_item() to initialize default and focused item styles. Sets styles_initialized flag to true.
*/
static void init_item_styles(void) {
    if (_TESTING) ESP_LOGI(TAG, "In init_item_styles()");
    if (styles_initialized) return;

    lv_style_init(&def_item_style);
    lv_style_set_bg_color(&def_item_style, lv_color_make(0, 0, 0));
    lv_style_set_bg_opa(&def_item_style, LV_OPA_COVER);
    lv_style_set_text_color(&def_item_style, lv_color_white());

    lv_style_init(&hover_item_style);
    lv_style_set_bg_color(&hover_item_style, lv_color_make(255, 255, 255));
    lv_style_set_bg_opa(&hover_item_style, LV_OPA_COVER);
    lv_style_set_text_color(&hover_item_style, lv_color_black());

    styles_initialized = true;
}

static void update_sidebar_state(bool show_sidebar) {
    if (show_sidebar) {
        lv_obj_set_size(scr_container, LV_HOR_RES - SIDEBAR_W, LV_VER_RES - STATUS_BAR_H);
    } else {
        lv_obj_set_size(scr_container, LV_HOR_RES, LV_VER_RES - STATUS_BAR_H);
    }

    lv_obj_set_flag(sidebar, LV_OBJ_FLAG_HIDDEN, !show_sidebar);
    sidebar_shown = show_sidebar;
}

/*
A callback function that calls load_system_state() as defined in lcd.c to update the gui_task with the desired GUI state change.
Called from create_sidebar_item().
If encoder button is pressed but the sidebar is not being shown, this function will show the sidebar and not cause any state change.
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

/*
Reusable function for creating each sidebar item with button-like styling.
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
    lv_obj_set_style_border_width(item, 2, 0);
    lv_obj_set_style_border_color(item, lv_color_white(), 0);
    lv_obj_set_style_border_opa(item, LV_OPA_COVER, 0);

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


/*
Create LVGL objects that function as the main menu sidebar
*/
void create_sidebar(void) {
    if (_TESTING) ESP_LOGI(TAG, "In create_sidebar()");

    if (!root_scr) {
        create_root_screen();
    }

    // Create the sidebar container
    sidebar = lv_obj_create(root_scr);

    // Size & alignment
    lv_obj_set_size(sidebar, SIDEBAR_W, LV_VER_RES - STATUS_BAR_H);
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
    lv_obj_set_style_bg_color(sidebar, lv_color_make(200, 200, 200), 0);
    lv_obj_set_style_bg_opa(sidebar, LV_OPA_COVER, 0);

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

// -------------------------
// Boot Screen
// -------------------------
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

/*
Callback to exit the system from boot state and enter the nominal use state.
*/
static void boot_bar_completed_cb() {
    // Replace boot screen with main root screen container
    lv_screen_load(root_scr);
    // Switch to main menu GUI state
    load_system_state(GUI_MAIN);
}

/*
Increments the value of the boot bar and calls boot_bar_completed_cb when done.
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

/*
Turns on LCD backlight, loads the boot screen object, and starts the boot bar animation.
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

    lv_obj_t *text = lv_label_create(main_scr);
    lv_label_set_text(text, "Welcome to PulsePioneer\n\n<Insert Setup Instructions Here>");
    lv_obj_set_style_text_color(text, lv_color_black(), 0);
    lv_obj_set_style_text_align(text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(text, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_flag(main_scr, LV_OBJ_FLAG_HIDDEN);

    if(_TESTING) ESP_LOGI(TAG, "Returning from create_main_screen()");
}

void show_main_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In show_main_menu()");

    // Create screen if not previously created
    if (!main_scr) {
        create_main_screen();
    }

    // Show the screen
    lv_obj_set_flag(main_scr, LV_OBJ_FLAG_HIDDEN, false);

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

static void add_waveform_plot() {
    if(_TESTING) ESP_LOGI(TAG, "In add_waveform_plot()");
    if (!waveform_ptr) {
        ESP_LOGE(TAG, "Waveform_ptr has not be initialized. Call create_ECG_screen() prior to add_waveform_plot().");
        return;
    }

    if (!waveform_ptr->ch1) {
        waveform_ptr->ch1 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    }
    if (!waveform_ptr->ch2) {
        waveform_ptr->ch2 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    }
    if (!waveform_ptr->ch3) {
        waveform_ptr->ch3 = lv_chart_add_series(waveform_ptr->chart, lv_palette_main(LV_PALETTE_LIGHT_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    }
}

lv_obj_t *get_ecg_scr_label() {
    return ecg_scr_label;
}

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
    lv_chart_set_point_count(waveform.chart, 500);
    
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

void show_ECG_screen(void) {
    if (_TESTING) ESP_LOGI(TAG, "In show_ECG_screen()");
    if (!ecg_scr) {
        create_ECG_screen(1);
    }
    // Show the ECG screen
    lv_obj_set_flag(ecg_scr, LV_OBJ_FLAG_HIDDEN, false);
    // Resume the ECG data streaming task
    vTaskResume(ecg_stream_task_handle);
    if (_TESTING) ESP_LOGI(TAG, "ecg_stream_task() resumed from show_ECG_screen().");
}

/*
Container function for initializing all main GUI screens using LVGL.
*/
void create_LVGL_screens() {
    if (_TESTING) ESP_LOGI(TAG, "In create_LVGL_screens()");
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

