#include <gtk/gtk.h>


void
on_window_exit                         (GtkObject       *object,
                                        gpointer         user_data);

void
on_new1_activate                       (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_open1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_save1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_save_as1_activate                   (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_quit1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_cut1_activate                       (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_copy1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_paste1_activate                     (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_delete1_activate                    (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

void
on_about1_activate                     (GtkMenuItem     *menuitem,
                                        gpointer         user_data);

gboolean
on_drawingarea1_configure_event        (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data);

gboolean
on_drawingarea1_expose_event           (GtkWidget       *widget,
                                        GdkEventExpose  *event,
                                        gpointer         user_data);

gboolean
on_drawingarea1_motion_notify_event    (GtkWidget       *widget,
                                        GdkEventMotion  *event,
                                        gpointer         user_data);

gboolean
on_drawingarea1_button_release_event   (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data);

void
on_entry1_changed                      (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_entry2_changed                      (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_entry3_changed                      (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_startx_value_changed                (GtkRange        *range,
                                        gpointer         user_data);

void
on_starty_value_changed                (GtkRange        *range,
                                        gpointer         user_data);

void
on_start_theta_value_changed           (GtkRange        *range,
                                        gpointer         user_data);

void
on_entry4_changed                      (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_entry5_changed                      (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_entry6_changed                      (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_targetx_value_changed               (GtkRange        *range,
                                        gpointer         user_data);

void
on_targety_value_changed               (GtkRange        *range,
                                        gpointer         user_data);

void
on_target_theta_value_changed          (GtkRange        *range,
                                        gpointer         user_data);

void
on_pixel_size_changed                  (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_pixels_per_tile_changed             (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_reg_grid_changed                    (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_dist_node_changed                   (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_obstacle_radius_changed             (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_bridge_length_changed               (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_k_distance_changed                  (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_k_theta_changed                     (GtkEditable     *editable,
                                        gpointer         user_data);

void
on_tracking_distance_value_changed     (GtkRange        *range,
                                        gpointer         user_data);

void
on_Linea_speed_value_changed           (GtkRange        *range,
                                        gpointer         user_data);

void
on_safety_distance_value_changed       (GtkRange        *range,
                                        gpointer         user_data);

void
on_connect_to_player_clicked           (GtkButton       *button,
                                        gpointer         user_data);

void
on_path_plan_released                  (GtkButton       *button,
                                        gpointer         user_data);

void
on_generate_cspace_released            (GtkButton       *button,
                                        gpointer         user_data);

void
on_follow_path_released                (GtkButton       *button,
                                        gpointer         user_data);

void
on_stop_following_released             (GtkButton       *button,
                                        gpointer         user_data);
void
on_simulate_radio_toggled              (GtkToggleButton *togglebutton,
                                        gpointer         user_data);

void
on_wheelchair_radio_toggled            (GtkToggleButton *togglebutton,
                                        gpointer         user_data);
