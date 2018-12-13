import rospy
import intera_interface
#import <mitchs CV> as weedFinder
#import <claires frontend> as frontEnd
#import <jason+thadeus movement calls> as ROScalls
import sawyer_python_file as ROScalls
import os.path

print(os.path.abspath(rospy.__file__))


"""
# Main function outline:
#             ~ INSTRUCTIONS ~                                     ~ ROUGH STATE MACHINE ~
# 1: Get instruction input from Claire's frontend            Decide where it will need to move |
# 2: Decode into cell-by-cell instructions                                                     |
# 3: Check first cell in instruction list for weeds                                            v   ^
# 4: Pull weed if necessary                                  Move accordingly |                    |
# 5: Execute cell instruction                                                 v                    |
# 6: Repeat steps 3-5 for all instructions                   Loop until no more instructions ------|


# Steps 4 and 5 have their own subroutines...

# Step 4: Pulling a weed from specified cell (this method will also be used to get rid fo an "old" plant)

#             ~ INSTRUCTIONS ~                          ~ ROUGH STATE MACHINE ~
# 1: Move from home -> hover over plant cell             Move to grab weed |
# 2: Move from plant hover -> into plant base                              v
# 3: Close grippers around weed                          Grab weed
# 4: Move from plant base -> plant hover                 Move to disposal area |
# 5: Move from plant hover -> home                                             |
# 6: Move from home -> disposal area                                           v
# 7: Open grippers to release weed                       Dispose of weed
# 8: Move from disposal -> home                          Return home

# Step 5: Execute cell instruction (plant new plant)
#             ~ INSTRUCTIONS ~                                       ~ ROUGH STATE MACHINE ~
# 1: Move from home -> hover over seed holder                          Move to grab seed |
# 2: Move from holder hover -> hover over specified seed slot                            |
# 3: Move from slot hover -> around seed (fake plant) head                               v
# 4: Close grippers to grab seed                                       Grab seed
# 5: Move from around seed -> slot hover                               Move to planting area |
# 6: Move from slot hover -> holder hover                                                    |
# 7: Move from holder hover -> home                                                          |
# 8: Move from home -> hover over plant cell                                                 |
# 9: Move from plant hover -> into plant base                                                v
# 10: Open grippers to release seed                                    Plant seed
# 11: Move from plant base -> plant hover                              Return to home |
# 12: Move from plant over -> home                                                    v

# The home->hover->target->hover->home model prevents any unpredictable movements of the Sawyer arm
# by breaking up each movement into smaller sections to limit the degrees of freedom accessed at any one time.
# Sawyer does *not* have a map of the planter layout, nor are we going to use CV to make one.
# This serves as a form of "manual" collision detection, because we are relying on inverse kinematics taking us to
# known waypoints, but do not have knowledge about obstacles that may lie in between.
# Feature of the model we have laid out:
# - All movements will start and end at home position
#    - No ambiguity in environmental variables when states are transitioned
#    - Will always allow for weed detection CV to have an unblocked view
# - Hover->target->hover always restricts final action to vertical movement ONLY
#    - Presence of lateral obstacles is hard to predict as positions of seeds/plants changes over time
#    - Makes gripping tasks as predictable as possible
# - All movement paths are palindormic, in the sense that they are played forwards then backwards
#    - Maintains predictability in all pathing scenarios
"""



# See pseudocode for Step 5 in comment block at top of file
class PlantInstruction:
    def __init__(self, seed_to_plant, cell_to_plant):
        self.seed_to_plant = seed_to_plant
        self.cell_to_plant = cell_to_plant

        
    def get_sawyer_movements(self):
        movement_list = []
        
        return movement_list
        """
        # TODO
        movement_list.append(SawyerMovement(ROScalls.get_current_position(limb), ROScalls.home))
        # Step 1: Move from home -> hover over seed holder
        movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_seeds))
        # grab the first seed
        movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.down_seed_1))

        movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.down_seed_1))
        

        # Step 2: Move from holder hover -> hover over specified seed slot
        movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_1))
        # Step 3: Move from slot hover -> around seed (fake plant) head
        movement_list.append(SawyerMovement(ROScalls.down_plot_1, ROScalls.hover_plot))
        # Step 4: Close grippers to grab seed
        # movement_list.append(GripperMovement(ROScalls.gripperactions.close))
        # Step 5: Move from around seed -> slot hover 
        movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_2))
        # Step 6: Move from slot hover -> holder hover 
        movement_list.append(SawyerMovement(ROScalls.down_plot_2, ROScalls.hover_plot))
        # Step 7: Move from holder hover -> home 
        movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_3))
        # Step 8: Move from home -> hover over plant cell
        movement_list.append(SawyerMovement(ROScalls.down_plot_3, ROScalls.hover_plot))
        # Step 9: Move from plant hover -> into plant base 
        movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.down_plot_4))
        # Step 10: Open grippers to release seed
        movement_list.append(SawyerMovement(ROScalls.down_plot_4, ROScalls.hover_plot))
        # Step 11: Move from plant base -> plant hover
        # movement_list.append()
        # Step 12: Move from plant over -> home
        movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.home))
        return movement_list
        """
    
    #plot functions
    #call the previous heiarchy before
    # p is before the function to indicate that it is a planting function
    # w is infront a function to indicate that it is a weeding function

    #from home to hover
    def p_home_to_hover_plot(self):
    	#initialize the movements needed 
    	movement_list = []
		#go down
		movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_plot))
		return movement_list

	#hover plot to home
	def p_home_to_hover_plot(self):
		#initialize the movements needed 
		movement_list = []
		#go down
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.home))
		return movement_list

	#hover down to the plots
	def p_down_hover_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_1))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def p_down_hover_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_2))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def p_down_hover_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_3))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def p_down_hover_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_4))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	#up hover from the plots
	def p_up_hover_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_1, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists
	
	def p_up_hover_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_2, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def p_up_hover_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_3, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def p_up_hover_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_4, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists
	
	#go down to the plant
	def p_down_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_1, ROScalls.down_plot_1))
		#plant whatever then go up
		return movement_lists

	def p_down_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_2, ROScalls.down_plot_2))
		#plant whatever then go up
		return movement_lists

	def p_down_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_3, ROScalls.down_plot_3))
		#plant whatever
		return movement_lists

	def p_down_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_4, ROScalls.down_plot_4))
		#plant whatever
		return movement_lists

	#from the plot to the hover plot
	def p_up_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_1, ROScalls.hover_plot_1))
		#plant whatever then go up
		return movement_lists

	def p_up_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_2, ROScalls.hover_plot_2))
		#plant whatever then go up
		return movement_lists

	def p_up_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_3, ROScalls.hover_plot_3))
		#plant whatever then go up
		return movement_lists

	def p_up_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_4, ROScalls.hover_plot_4))
		#plant whatever then go up
		return movement_lists

	#functions for the seeds
	#go home to the hover seeds
	def p_home_to_hover_seeds(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_seeds))
		return movement_lists

	#from seeds to hover home
	def p_hover_seeds_to_home(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.home))
		return movement_lists

	#from general hover position to the seds
	def p_down_hover_seed_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.hover_seed_1))
		return movement_lists

	def p_down_hover_seed_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.hover_seed_2))
		return movement_lists

	def p_down_hover_seed_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.hover_seed_3))
		return movement_lists

	def p_down_hover_seed_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seeds, ROScalls.hover_seed_4))
		return movement_lists

	#from the hover seed positions to the new positions
	def p_up_hover_seed_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_1, ROScalls.hover_seeds))
		return movement_lists

	def p_up_hover_seed_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_2, ROScalls.hover_seeds))
		return movement_lists

	def p_up_hover_seed_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_3, ROScalls.hover_seeds))
		return movement_lists

	def p_up_hover_seed_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_4, ROScalls.hover_seeds))
		return movement_lists

	#from the specific seed hover to going down to get the seeds
	def p_down_seed_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_1, ROScalls.down_seed_1))
		return movement_lists

	def p_down_seed_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_2, ROScalls.down_seed_2))
		return movement_lists

	def p_down_seed_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_3, ROScalls.down_seed_3))
		return movement_lists

	def p_down_seed_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_seed_4, ROScalls.down_seed_4))
		return movement_lists

	#going from the down seeds to hover above the seeds
	def p_up_seed_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_seed_1, ROScalls.hover_seed_1))
		return movement_lists

	def p_up_seed_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_seed_2, ROScalls.hover_seed_2))
		return movement_lists

	def p_up_seed_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_seed_3, ROScalls.hover_seed_3))
		return movement_lists

	def p_up_seed_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_seed_4, ROScalls.hover_seed_4))
		return movement_lists

class WeedInstruction:
    def __init__(self, cell_to_weed):
        self.cell_to_weed = cell_to_weed
        
    def get_sawyer_movements(self):
        movement_list = []
        #could this be what calls the list?
        #the original code did not work at runntime, which is how we are going to have to do everything
        #unless we implement some multithreaded system
        
        # TODO
        # Step 1: Move from home -> hover over plant cell
        #movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_plot))
        # Step 2: Move from plant hover -> into plant base
        # Step 3: Close grippers around weed  
        # Step 4: Move from plant base -> plant hover 
        # Step 5: Move from plant hover -> home 
        # Step 6: Move from home -> disposal area aa
        # Step 7: Open grippers to release weed 
        # Step 8: Move from disposal -> home

	# p is before the function to indicate that it is a planting function
	# w is infront a function to indicate that it is a weeding function
	
	def w_home_to_hover_dump(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.home, ROScalls.hover_dump))
		#guess this should be where wee call the shit to dump the plant
		return movement_lists
    def w_hover_dump_to_home(self):
    	movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_dump, ROScalls.home))
		#guess this should be where wee call the shit to dump the plant
		return movement_lists
		#hover plot to home
    def w_home_to_hover_plot(self):
    	#initialize the movements needed 
    	movement_list = []
		#go down
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.home))
		return movement_list

	#hover down to the plots
	def w_down_hover_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_1))
		#should we implement a call here?
		return movement_lists

	def w_down_hover_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_2))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def w_down_hover_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_3))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def w_down_hover_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot, ROScalls.hover_plot_4))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	#up hover from the plots
	def w_up_hover_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_1, ROScalls.hover_plot))
		return movement_lists
	
	def w_up_hover_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_2, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def w_up_hover_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_3, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists

	def w_up_hover_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_4, ROScalls.hover_plot))
		#plant whatever then go up
		#should we implement a call here?
		return movement_lists
	
	#go down to the plant
	def w_down_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_1, ROScalls.down_plot_1))
		#grab the weeb/object that is not needed to remove
		return movement_lists

	def w_down_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_2, ROScalls.down_plot_2))
		#grab the weeb/object that is not needed to remove
		return movement_lists

	def w_down_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_3, ROScalls.down_plot_3))
		#grab the weeb/object that is not needed to remove
		return movement_lists

	def w_down_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.hover_plot_4, ROScalls.down_plot_4))
		#grab the weeb/object that is not needed to remove
		return movement_lists

	#from the plot to the hover plot
	def w_up_plot_1(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_1, ROScalls.hover_plot_1))
		return movement_lists

	def w_up_plot_2(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_2, ROScalls.hover_plot_2))
		return movement_lists

	def w_up_plot_3(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_3, ROScalls.hover_plot_3))
		return movement_lists

	def w_up_plot_4(self):
		movement_list = []
		movement_list.append(SawyerMovement(ROScalls.down_plot_4, ROScalls.hover_plot_4))
		return movement_lists

        
class GripperMovement:
    def __init__(self, action):
        # This will be ROS-formatted gripper coordinates for opening/closing 
        self.action = action
        
    def get_ROS_call(self):
        # This will execute the relevant function from within ROScalls
        # TODO
        pass

class SawyerMovement:
    def __init__(self, start_pos, end_pos):
        # These are x/y coordinates returned by ROScalls library (Jason+Thadeus)
        self.start_pos = start_pos
        self.end_pos = end_pos
        
    def get_ROS_call(self, limb):
        # This will execute the relevant function from within ROScalls
        # TODO
        ROScalls.create_new_position(self.end_pos, limb)

if __name__ == '__main__':
    # In[3]:
    rospy.init_node("sawyer")
    # In[4]:
    limb = intera_interface.Limb('right')
    # In[5]:
    robot_state = intera_interface.RobotEnable()
    # In[6]:
    robot_state.enable()

    # Step 1: Get instructions from Claire's frontend
    plant_list = [0] #frontEnd.get_user_input()
    
    # Output format: plant_list = [p0, p1, p2, p3] or something similar...
    # |----|----|
    # | p0 | p1 |
    # |----|----| <- ...where pN represents what plant should go in that cell
    # | p2 | p3 |
    # |----|----|
    
    # Step 2: Decode into cell-by-cell instructions
    # Note that this is always done in order p0->...->p3

    #pseudocode, of course something will be done for each one, but this is the general idea
    # start
    # 	(something_in_plot_1) ? ((is_what_we_want) ? move_to_plot_2: weed_plot_1 -> plant_plot_1) : plant_plot_1
    # 	(something_in_plot_2) ? ((is_what_we_want) ? move_to_plot_3: weed_plot_2 -> plant_plot_2) : plant_plot_2
    # 	(something_in_plot_3) ? ((is_what_we_want) ? move_to_plot_4: weed_plot_3 -> plant_plot_3) : plant_plot_3
    # 	(something_in_plot_4) ? ((is_what_we_want) ? finish : weed_plot_4 -> plant_plot_4) : plant_plot_4
    # finish





    main_instruction_set = []
    for cell_number, seed in enumerate(plant_list):
        new_plant_movements = PlantInstruction(seed, cell_number)
        instruction_set = new_plant_movements.get_sawyer_movements()
        for movement in instruction_set:
            main_instruction_set.append(movement)
    
    for movement in main_instruction_set:
        #rospy.sleep(0.5)
        movement.get_ROS_call(limb)


        
    # Step 6: Repeat steps 3-5 for all instructions
    """
    for cell_number, main_instruction in enumerate(main_instruction_set):
        
        # Step 3: Check first cell in instruction list for weed with Mitch's CV
        weed_found = False #weedFinder.check_cell(cell_number)
        
        # Step 4: Pull weed if necessary
        if weed_found:
            weed_instruction = WeedInstruction(cell_number)
            weed_instruction.get_sawyer_movements()
            
        # Step 5: Execute cell instruction
        main_instruction.get_sawyer_movements()
    """
