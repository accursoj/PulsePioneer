/*
Functions related to creating and controlling the LVGL graphical user interface.
*/

#include "gui.h"

#define SIDEBAR_W 80

#define _TESTING 1

lv_obj_t *scr_container = NULL;

static lv_obj_t *root_scr = NULL;
static lv_obj_t *sidebar = NULL;
// static QueueHandle_t indev_queue = NULL;
static rotary_encoder_event_t enc_event;
static lv_indev_t *enc_dev = NULL;
static lv_group_t *sidebar_group = NULL;
static lv_style_t def_item_style;
static lv_style_t hover_item_style;
static bool styles_initialized = false;

const char *TAG = "gui.c";


void create_root_screen(void) {
    root_scr = lv_obj_create(NULL);

    lv_obj_remove_flag(root_scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(root_scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(root_scr, LV_OPA_COVER, 0);

    lv_screen_load(root_scr);
}

lv_obj_t *get_root_screen(void) {
    if (!root_scr) {
        create_root_screen();
    }
    return root_scr;
}

void create_scr_container(void) {
    if (!scr_container) {
        if (!root_scr) {
            create_root_screen();
        }
        scr_container = lv_obj_create(root_scr);
        lv_obj_align(scr_container, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_set_size(scr_container, LV_HOR_RES - SIDEBAR_W, LV_VER_RES);
        lv_obj_set_style_pad_all(scr_container, 0, 0);
        lv_obj_set_style_border_width(scr_container, 0, 0);
        lv_obj_set_style_bg_opa(scr_container, LV_OPA_TRANSP, 0);

        lv_obj_remove_flag(scr_container, LV_OBJ_FLAG_SCROLLABLE);
    }
}


/*
Callback function for an encoder-based LVGL input device.
Called from init_sidebar_input().
*/
static void enc_read(lv_indev_t *indev, lv_indev_data_t *data) {
    // if (!indev_queue) {
    //     indev_queue = xQueueCreate(8, sizeof(rotary_encoder_event_t));
    // }
    // if (xQueueReceive(indev_queue, &enc_event, 0) != pdFALSE) {
    if (xQueueReceive(forwarded_enc_queue, &enc_event, 0) != pdFALSE) {
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

/*
A callback function that calls load_system_state() as defined in lcd.c to update the gui_task with the desired GUI state change.
Called from create_sidebar_item().
*/
static void sidebar_item_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    system_state_t state = (system_state_t)lv_event_get_user_data(e);

    if (_TESTING) ESP_LOGI(TAG, "sidebar_item_event_cb() got passed a LV_EVENT_CLICKED: %d", (state));

    load_system_state(state);
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
    lv_obj_set_size(sidebar, SIDEBAR_W, LV_VER_RES);
    lv_obj_align(sidebar, LV_ALIGN_RIGHT_MID, 0, 0);

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
}

/*
Receive encoder event from a hardware input queue and adds it to the queue for lv_indev events
*/
// void lv_indev_pass_enc_event(rotary_encoder_event_t *enc_event) {
//     if (_TESTING) ESP_LOGI(TAG, "In lv_indev_pass_enc_event()");

//     if (!indev_queue) {
//         indev_queue = xQueueCreate(8, sizeof(rotary_encoder_event_t));
//     }
//     xQueueSend(indev_queue, enc_event, 0);
// }