import <mitchs CV> as weedFinder
import <claires frontend> as frontEnd
import <jason+thadeus movement calls> as ROScalls

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
        
        # TODO
        # Step 1: Move from home -> hover over seed holder
        movement_list.append(SawyerMovement(ROScalls.positions.home, ROScalls.positions.seed_holder_hover))
        # Step 2: Move from holder hover -> hover over specified seed slot
        movement_list.append()
        # Step 3: Move from slot hover -> around seed (fake plant) head
        movement_list.append()
        # Step 4: Close grippers to grab seed
        movement_list.append(GripperMovement(ROScalls.gripperactions.close))
        # Step 5: Move from around seed -> slot hover 
        movement_list.append()
        # Step 6: Move from slot hover -> holder hover 
        movement_list.append()
        # Step 7: Move from holder hover -> home 
        movement_list.append()
        # Step 8: Move from home -> hover over plant cell
        movement_list.append()
        # Step 9: Move from plant hover -> into plant base 
        movement_list.append()
        # Step 10: Open grippers to release seed
        movement_list.append()
        # Step 11: Move from plant base -> plant hover
        movement_list.append()
        # Step 12: Move from plant over -> home
        movement_list.append()
        

class WeedInstruction:
    def __init__(self, cell_to_weed):
        self.cell_to_weed = cell_to_weed
        
    def get_sawyer_movements(self):
        movement_list = []
        
        # TODO
        # Step 1: Move from home -> hover over plant cell
        movement_list.append(SawyerMovement(ROScalls.positions.home, ROScalls.positions.seed_holder_hover))
        # Step 2: Move from plant hover -> into plant base
        # Step 3: Close grippers around weed  
        # Step 4: Move from plant base -> plant hover 
        # Step 5: Move from plant hover -> home 
        # Step 6: Move from home -> disposal area 
        # Step 7: Open grippers to release weed 
        # Step 8: Move from disposal -> home
   
        
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
        self.end_post = end_pos
        
    def get_ROS_call(self):
        # This will execute the relevant function from within ROScalls
        # TODO
        pass

if __name__ == '__main__':

    # Step 1: Get instructions from Claire's frontend
    plant_list = frontEnd.get_user_input()
    
    # Output format: plant_list = [p0, p1, p2, p3] or something similar...
    # |----|----|
    # | p0 | p1 |
    # |----|----| <- ...where pN represents what plant should go in that cell
    # | p2 | p3 |
    # |----|----|
    
    # Step 2: Decode into cell-by-cell instructions
    # Note that this is always done in order p0->...->p3
    main_instruction_set = []
    for cell_number, seed in enumerate(plant_list):
        main_instruction_set.append(PlantInstruction(seed, cell_number))
        
    # Step 6: Repeat steps 3-5 for all instructions
    for cell_numer, main_instruction in enumerate(main_instruction_set):
        
        # Step 3: Check first cell in instruction list for weed with Mitch's CV
        weed_found = weedFinder.check_cell(cell_number)
        
        # Step 4: Pull weed if necessary
        if weed_found:
            weed_instruction = WeedInstruction(cell_number)
            weed_instruction.get_sawyer_movements()
            
        # Step 5: Execute cell instruction
        main_instruction.get_sawyer_movements()
