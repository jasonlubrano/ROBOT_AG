#current to go home
movement_list.append(SawyerMovement(ROScalls.get_current_position(limb), ROScalls.home))

#the seed movements
movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_seeds))
movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.home))

movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.down_seed_1))
movement_list.append(SawyerMovement(ROScalls.down_seed_1, ROScalls.hover_plot))

movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.down_seed_2))
movement_list.append(SawyerMovement(ROScalls.down_seed_2, ROScalls.hover_plot))

movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.down_seed_3))
movement_list.append(SawyerMovement(ROScalls.down_seed_3, ROScalls.hover_plot))

movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.down_seed_4))
movement_list.append(SawyerMovement(ROScalls.down_seed_4, ROScalls.hover_plot))

#plot movements
#calls to move from home to the plot
movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_plot))
movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.home))

movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_1))
movement_list.append(SawyerMovement(ROScalls.down_plot_1, ROScalls.hover_plot))

movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_2))
movement_list.append(SawyerMovement(ROScalls.down_plot_2, ROScalls.hover_plot))

movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_3))
movement_list.append(SawyerMovement(ROScalls.down_plot_3, ROScalls.hover_plot))

movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_4))
movement_list.append(SawyerMovement(ROScalls.down_plot_4, ROScalls.hover_plot))

#dump movements
movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_dump))
movement_list.append(SawyerMovement(ROScalls.hover_dump, ROScalls.home))