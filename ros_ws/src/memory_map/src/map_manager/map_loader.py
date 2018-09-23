#!/usr/bin/python
import os
import yaml
import rospy

from memory_definitions.srv import GetDefinition

class LoadingHelpers():
    '''
    Helpers static class. Provides validation methods for easier and
    cleaner YML verification handling inside the classes.
    '''

    @staticmethod
    def checkKeysExist(checkdict, *keys_required):
        '''
        Checks if the dict given has all of the keys given in keys_required.
        Pops a ROS error and stops the node if the condition is not verified.
        '''
        for k in keys_required:
            if k not in checkdict.keys():
                m = "Missing required '{}' element in Map YML description file. Couldn't load Map.".format(k)
                rospy.logerr(m)
                raise ValueError(m)

    @staticmethod
    def checkValueValid(value, *values_required):
        '''
        Checks if the the value given corresponds to one of the possibilities given in values_required.
        Pops a ROS error and stops the node if the condition is not verified.
        '''
        if value not in values_required:
            m = "Element value '{}' not valid, must be in '{}'. Couldn't load Map.".format(value, values_required)
            rospy.logerr(m)
            raise ValueError(m)

    @staticmethod
    def mergeDicts(a, b): # if a key exists on both dicts, will take b's value.
        c = a.copy()   # start with x's keys and values
        c.update(b)    # modifies z with y's keys and values & returns None
        return c


class MapLoader():
    @staticmethod
    def loadFile(filename):
        '''
        Gets the YML Map description file from the specified method
        Please change the method correspondingly to what is currently used in your package.
        '''
        return MapLoader.loafYamlFromDescriptionModule(filename)

    @staticmethod
    def loadYamlFromFile(filename): # DEPRECATED
        '''
        Loads the description file simply by getting the file in disk.
        The file MUST be in the package's directory to avoid any problems.
        '''
        return MapLoader._json_from_file(os.path.dirname(__file__) + "/../../def" + filename)

    @staticmethod
    def loafYamlFromDescriptionModule(filename):
        '''
        Loads the description file by gettign it from the 'memory/definitions' ROS package.
        '''
        get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)
        try:
            get_def.wait_for_service(timeout = 2)
        except:
            rospy.logerr("FATAL Could not contact definitions service, timeout reached. Aborting.")
            raise Exception()

        try:
            res = get_def('map/' + filename)
            if not res.success:
                rospy.logerr("Error when fetching '{}' definition file".format(filename))
                raise Exception()
            return MapLoader._json_from_file(res.path)
        except rospy.ServiceException as exc:
            rospy.logerr("Unhandled error while getting def file: {}".format(str(exc)))
            raise Exception()

    @staticmethod
    def _json_from_file(path):
        with open(path, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                rospy.logerr("Could not load map YML file : " + str(exc))
