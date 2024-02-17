import json
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from std_srvs.srv import Trigger
from core.msg import Cam
from core.srv import AddCamera

# NOTE:
        # This program has 2 config dictionaries. 
            # Standard (self.std_camera_config), and Master (self.master_config).
        # Standard can be changed via GUI parameters. Master cannot.
        # Master is referenced in the process of changing cameras.
        # Master always matches what's currently written in the JSON file.
        # At shutdown, depending on parameters, Standard is written to the JSON file.
        # Neither of these should  be confused with self.active_cameras.
        # self.active cameras contains the ips of all of the currently connected cameras.
        # The config files contain all of the currently configured cameras.

class Camera_Switcher(Node):

    def __init__(self):
        super().__init__('camera_switcher')

        self.max_cameras = 8 # This is not a hard limit. Can change.

        self.log = self.get_logger() # Quick reference for ROS logging

        # Stores IPs of active cameras
        # Filling unused slots with empty dictionaries
        self.active_cameras = []
        self.current_camera_index = 0

        # Number of unnamed cameras. Used in generating IDs for new cams.
        self.unnamed_cams = 0

        # For remembering previous camera button presses
        # This way we only try to change when the button is first pushed
        # And not repeatedly when it is held down
        self.cached_button_input = [0, 0, 0, 0]
        self.cached_camera_index = None

        # Define config file path
        self.config_path = "/home/jhsrobo/corews/src/pilot_gui/cam_config.json"

        # Opens the JSON config file.
        self.check_config_integrity()
        self.open_config() # Creates both Standard and Master Config.

        # Set up a service for accepting new cameras from find_cameras
        self.camera_adder = self.create_service(AddCamera, "add_camera", self.add_camera_callback)

        # Set up a service for returning the first camera to view_cameras
        self.first_camera_srv = self.create_service(Trigger, 'first_camera', self.first_camera_callback)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.change_cam_callback, 10)
        self.camera_pub = self.create_publisher(Cam, "active_camera", 10)

        self.declare_parameter('save_changes_on_shutdown', False)
        self.add_on_set_parameters_callback(self.parameters_callback)


    # Edit the camera config according to parameters
    # Get the name of the camera that was changed
    # Copy the entry from the intial_camera_config to the new assigned index
    # The entry is written to the new index in camera_config
    # And changes are written permanently depending on parameter
    def parameters_callback(self, params):
        for param in params:
            # List of parameters to ignore in this function
            if param.name not in ["use_sim_time", "permanent_changes", "save_changes_on_shutdown"]:
                new_index = param.value
                ID = param.name.replace('_index', '') # Get ID from parameter name
                master_index = int(self.get_master_index(ID))
                nickname = self.master_config[str(master_index)]["nickname"]

                # Update the config and the active cameras list
                self.delete_camera_entries(nickname)
                self.std_camera_config[str(new_index)] = self.master_config[str(master_index)]

                self.log_camera_assignment_change(nickname, new_index)

        return SetParametersResult(successful=True)


    # When the AddCamera service is requested (from find_cameras.py), 
    # add the attached IP to the camera dict.
    def add_camera_callback(self, request, response):

        # Check to see if this IP is pre-registered in the config file
        ip_index = self.get_std_index(request.ip)
        if ip_index is not None:
            pass
        else:
            self.create_config_entry(request.ip)
            ip_index = self.get_std_index(request.ip) # Get new index once it's been added to config

        self.active_cameras.append(request.ip)
        self.new_cam_parameter(ip_index)

        return response

    # When view_cameras starts up, it requests for us to publish the currently active camera
    def first_camera_callback(self, request, response):
        if len(self.active_cameras) == 0: # If no cams connected, return early
            response.success = False
            return response
        
        # Get the index of the first camera in active_cameras
        self.current_camera_index = self.get_master_index(self.active_cameras[0])
        self.cached_camera_index = self.current_camera_index

        camera_msg = self.create_camera_msg()
        self.camera_pub.publish(camera_msg)
        response.success = True
        return response


    # Monitor controller input for D-Pad press
    # Used to change active camera
    def change_cam_callback(self, joy):

        change = False

        if joy.buttons[4] or joy.buttons[5] or joy.buttons[6] or joy.buttons[7]:
            # Minor consequence of this logic is:
            # If multiple buttons are pressed, the furthest clockwise takes priority
            # No big deal tbh. About as reasonable a solution as any.
            
            desired_camera_index = False

            # Also, this cached input stuff is so that holding the button does not trigger this repeatedly.
            if joy.buttons[4] and not self.cached_button_input[0]: # Up button
                desired_camera_index = 1
                change = True
            if joy.buttons[5] and not self.cached_button_input[1]: # Right Button
                desired_camera_index = 2
                change = True
            if joy.buttons[6] and not self.cached_button_input[2]: # Down button
                desired_camera_index = 3
                change = True
            if joy.buttons[7] and not self.cached_button_input[3]: # Left Button
                desired_camera_index = 4
                change = True

            # If a button state has changed...
            if change: 
                # If there's an entry in the config file for this index...
                if str(desired_camera_index) in self.std_camera_config.keys():   
                    # If there's an active camera with that IP...
                    if self.std_camera_config[str(desired_camera_index)]["ip"] in self.active_cameras:
                        # If it's not the same IP as the last one we published...
                        if self.cached_camera_index is not desired_camera_index:
                            self.current_camera_index = desired_camera_index
                            camera_msg = self.create_camera_msg()
                            self.camera_pub.publish(camera_msg)

                    else: # If there is no active camera with that IP
                        self.log.warn("No camera mapped to that button")    
                else: # If there is no entry in the config file for this index:
                    self.log.warn("No camera mapped to that button")
            
        self.cached_button_input = [joy.buttons[4], joy.buttons[5], joy.buttons[6], joy.buttons[7]]
        self.cached_camera_index = self.current_camera_index


    # Function for logging important information after remapping cameras
    def log_camera_assignment_change(self, nickname, new_index):
        
        # Put together a list of the nicknames of active cameras
        nickname_list = []
        for ip in self.active_cameras:
            master_index = self.get_master_index(ip)
            nickname_list.append(self.master_config[master_index]["nickname"])
        self.log.info('')
        self.log.info("{} camera assigned to index {}".format(nickname, new_index))
        self.log.info("Assigned Camera List: {}".format(self.get_nickname_printout()))


    # Creates a new parameter for a camera
    def new_cam_parameter(self, index):

        # Define the parameter settings
        int_range = IntegerRange()
        int_range.from_value = 1
        int_range.to_value = self.max_cameras
        int_range.step = 1
        ID = self.std_camera_config[index]["ID"]
        param_name = '{}_index'.format(ID)

        descriptor = ParameterDescriptor(integer_range = [int_range])

        self.declare_parameter(param_name, int(index), descriptor)


    # Populate a camera msg with all of the info about the current camera
    def create_camera_msg(self):
        index = str(self.current_camera_index)
        camera_msg = Cam()
        camera_msg.index = int(index)
        camera_msg.ip = self.std_camera_config[index]["ip"]
        camera_msg.gripper = self.std_camera_config[index]["gripper"]
        camera_msg.nickname = self.std_camera_config[index]["nickname"]

        return camera_msg


    # Generate a readable version of the current active cameras
    def get_nickname_printout(self):
        printout = {}
        for index in self.std_camera_config:
            if index != 0:
                printout[index] = self.std_camera_config[index]["nickname"]
            else:
                printout[index] = " "
        return str(printout)


    # Read the existing camera config into a dictionary
    # JSON files are read into nested dictionaries.
    # Also, keep in mind that although the key values are numbers 1-4, they are strings, not ints.
    def open_config(self):
        with open(self.config_path) as f:
            self.std_camera_config = json.load(f)

        # Save a version of the config for refererence when cameras are changed during runtime
        with open(self.config_path) as f:
            self.master_config = json.load(f)


    # Checks if the config file still exists.
    # If it doesn't, it creates a new one.
    def check_config_integrity(self):
        if os.path.isfile(self.config_path): 
            return True
        else:
            empty_dict = {}
            self.log.info("cam_config.json not found")
            self.log.info("Creating new camera config")
            self.log.info("Edit pilot_gui/cam_config.json to save your settings")
            with open(self.config_path, "w") as f:
                json.dump(empty_dict, f)


    # Scans the standard config for a certain attribute in inner dictionaries
    # Returns the index of that attribute if found
    def get_std_index(self, value):
        for index in self.std_camera_config:
            for key in self.std_camera_config[index]:
                if self.std_camera_config[index][key] == value:
                    return index
        return None
    

    # Scans the master config for a certain attribute in inner dictionaries
    # Returns the index of that attribute if found
    def get_master_index(self, value):
        for index in self.master_config:
            for key in self.master_config[index]:
                if self.master_config[index][key] == value:
                    return index
        return None


    # These functions search their indices for empty dict keys
    # If none are found, they create a new one.
    def find_available_std_index(self):
        for key in self.std_camera_config:
            if self.std_camera_config[key] == 0:
                return key
        return str(len(self.std_camera_config) + 1)


    def find_available_master_index(self):
        for key in self.master_config:
            if self.master_config[key] == 0:
                return key 
        return str(len(self.master_config) + 1)


    # Creates a new entry in the config files for new cameras
    def create_config_entry(self, ip, gripper="Front", ID="unnamed", nickname = "Unnamed"):
        std_index = self.find_available_std_index() # Assign a new index
        master_index = self.find_available_master_index()
        
        # Give each unnamed cam a unique identifier
        if "unnamed" in ID or "Unnamed" in nickname:
            self.unnamed_cams +=1
        if ID == "unnamed":
            ID += str(self.unnamed_cams)
        if nickname == "Unnamed":
            nickname += str(self.unnamed_cams)
        
        self.std_camera_config[std_index] = { "ip" : ip,
                                      "gripper" : gripper,
                                      "nickname" : nickname,
                                      "ID" : ID }
        self.master_config[master_index] = { "ip" : ip,
                                      "gripper" : gripper,
                                      "nickname" : nickname,
                                      "ID" : ID }

        self.write_to_config()
        self.log.info("")
        self.log.info("Created new camera config entry")


    # Updates the contents of the config file with the contents of self.std_camera_config
    # Setting save_changes high means that it writes the std_camera_config to the JSON
    # This means that changes we made during runtime are reflected in the config file.
    # Otherwise, only new cameras will be added to the config file.
    def write_to_config(self, save_changes=False):
        config_json = json.dumps(self.master_config, indent=2)
        if save_changes: 
            config_json = json.dumps(self.std_camera_config, indent=2)
        
        with open(self.config_path, "w") as f:
            f.write(config_json)

    # Simple getter to run at shutdown
    def get_save_changes (self):
        return self.get_parameter("save_changes_on_shutdown").value
    

    def delete_camera_entries(self, nickname):
        target_index = self.get_std_index(nickname)
        while target_index is not None:
            self.std_camera_config.pop(str(target_index))
            target_index = self.get_std_index(nickname)


def main(args=None):
    rclpy.init(args=args)

    camera_switcher = Camera_Switcher()

    # Runs the program until shutdown is recieved
    try: rclpy.spin(camera_switcher)
    except KeyboardInterrupt:
        save_changes = camera_switcher.get_save_changes()
        camera_switcher.write_to_config(save_changes)

    # On shutdown, kill node
    camera_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
