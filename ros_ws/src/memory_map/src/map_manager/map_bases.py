#!/usr/bin/python
import rospy
import json

class SetMode():
    MODE_ADD     = 0
    MODE_REPLACE = 1
    MODE_DELETE  = 2

class MapElement(object):
    def get(self, requestpath):
        raise NotImplementedError("This is the super class. Needs to be overwritten from the child class.")
    def set(self, requestpath, mode):
        raise NotImplementedError("This is the super class. Needs to be overwritten from the child class.")


# class ListManager(MapElement): # Not used for now
#     def __init__(self, classdef, initdict):
#         self.Elements = []
#         for k in initdict.keys():
#             self.Elements.append(classdef(initdict[k]))


class DictManager(MapElement):
    def __init__(self, elemdict):
        self.Dict = elemdict if elemdict is not None else {}
        if self.Dict:
            for k in self.Dict.keys():
                if isinstance(k, dict):
                    raise ValueError("Inner dicts as values NOT allowed. '{}' has a dict inside. Must be initialized beforehand.".format(k))

    def toList(self):
        return self.Dict.values()

    def toDict(self, hide_private = False, recursive = False):
        if recursive:
            d = {}
            for item in self.Dict:
                if isinstance(self.Dict[item], DictManager):
                    if (hide_private and item[0] != "_") or not hide_private:
                        d[item] = self.Dict[item].toDict(hide_private, recursive = True)
                else:
                    if (hide_private and item[0] != "_") or not hide_private:
                        d[item] = self.Dict[item]
            return d
        else:
            for k in self.Dict.values():
                if isinstance(k, DictManager):
                    rospy.logerr(("    ERROR Trying to transform a DictManager that has other DictManagers inside."
                                " Use '*' at the end to get a recursively-constructed subdict."))
                    return None
            return self.Dict

    def get(self, requestpath):
        if isinstance(requestpath, str):
            requestpath = RequestPath(requestpath)
            if not requestpath.Valid: return None
        keyname = requestpath.getNextKey()
        if requestpath.isLast():
            if "," in keyname:
                keys = keyname.split(",")
                d = {}
                for k in keys:
                    if not isinstance(self.Dict[k], DictManager):
                        d[k] = self.Dict[k]
                    else:
                        rospy.logerr("    GET Request failed : Must include a '*' or '^' dict operator at the end to get a full dict json or object.")
                return d
            elif keyname in self.Dict.keys():
                if not isinstance(self.Dict[keyname], DictManager):
                    return self.Dict[keyname]
                rospy.logerr("    GET Request failed : Must include a '*' or '^' dict operator at the end to get a full dict json or object.")
            elif keyname == '^':
                return self
                # rospy.logerr("    GET Request failed : Asked to get a DictManager object with key operator '^' but '{}' points to a '{}' object.".format(keyname, type(self.Dict[keyname])))
            elif keyname == '*':
                return self.toDict(hide_private = True, recursive = True)
            else:
                rospy.logerr("    GET Request failed : Unrecognized last key dict operator '{}'.".format(keyname))
        else:
            if keyname in self.Dict.keys():
                if isinstance(self.Dict[keyname], DictManager):
                    return self.Dict[keyname].get(requestpath)
                else:
                    return self.Dict[keyname]
            elif keyname == "*":
                d = {}
                current_level = requestpath.Counter
                for item in self.Dict:
                    if isinstance(self.Dict[item], DictManager):
                        d[item] = self.Dict[item].get(requestpath)
                        requestpath.Counter = current_level
                    else:
                        d[item] = self.Dict[item]
                return d
            else:
                rospy.logerr("    GET Request failed : Couldn't recognize key or find element '{}'.".format(keyname))

    def set(self, requestpath, mode, instance = None):
        if isinstance(requestpath, str):
            requestpath = RequestPath(requestpath)
            if not requestpath.Valid: return None
        keyname = requestpath.getNextKey()

        if requestpath.isLast():
            if mode == SetMode.MODE_ADD:
                return self._set_add(keyname, instance)
            elif mode == SetMode.MODE_REPLACE:
                return self._set_replace(keyname)
            elif mode == SetMode.MODE_DELETE:
                return self._set_remove(keyname)
        else:
            if '=' in keyname:
                rospy.logerr("    Request failed : '=' assign operator must only be applied on the last path key.")
                return False
            if keyname in self.Dict:
                return self.Dict[keyname].set(requestpath, mode, instance)

    def _set_add(self, lastkey, instance = None):
        # e.g. /objects/cube_42:MapObject={"type": "object", "etc": True}
        if instance is None:
            request, new_dict = lastkey.split('=', 1) # split at first '='.
            keyname, class_type = request.split(':')
            new_dict = json.loads(new_dict)

            if keyname in self.Dict:
                rospy.logerr("    ADD Request failed : key '{}' already exists in dict.".format(keyname))
                return False

            sc = DictManager.__subclasses__()
            for c in sc:
                if class_type == c.__name__:
                    self.Dict[keyname] = DictManager(new_dict)
                    return True
            rospy.logerr("    ADD Request failed : class type '{}' not recognized.".format(class_type))
            return False
        else: # add a DictManager (intern only, e.g. coming from MapTransfer).
            self.Dict[lastkey] = instance
            return True


    def _set_replace(self, lastkey):
        # e.g. /objects/cube_14/position/x=0.42,y=0.84,a=3.14
        assignments = []
        for s in lastkey.split(','):
            if s.count("=") != 1:
                rospy.logerr("    SET Request failed : invalid key part '{}', must have one '=' operator.".format(s))
                return False
            key, new_value = s.split("=")
            if not key in self.Dict:
                rospy.logerr("    SET Request failed : key '{}' does not already exist in dict.".format(key))
                return False
            if not isinstance(self.Dict[key], DictManager):
                try:
                    new_value = type(self.Dict[key])(new_value)
                    assignments.append((key, new_value))
                except (TypeError, ValueError):
                    rospy.logerr("    SET Request failed : new value '{}' not castable to the old value '{}''s type.".format(new_value, self.Dict[key]))
                    return False
                except KeyError:
                    rospy.logerr("    SET Request failed : Couldn't find existing key '{}'.".format(key))
                    return False
            else:
                raise ValueError("    SET Request failed : Can't SET a whole DictManager to a new value.")

        for assignment in assignments:
            self.Dict[assignment[0]] = assignment[1]
        return True

    def _set_remove(self, lastkey):
        # e.g. /objects/cube_12
        if not lastkey in self.Dict:
            rospy.logerr("    DEL Request failed : key '{}' does not already exist in dict.".format(lastkey))
            return False
        del self.Dict[lastkey]
        return True

    def transform(self, codes):
        success = True
        for elem in self.Dict:
            if isinstance(self.Dict[elem], DictManager):
                success = min(success, self.Dict[elem].transform(codes))
        return success



class RequestPath():
    def __init__(self, pathstring):
        self.pathstring = pathstring

        self.Valid = True
        if not len(pathstring):
            rospy.logerr("Invalid path : empty path.")
            self.Valid = False
        if pathstring[-1] == '/':
            rospy.logerr("Invalid path : must not end with '/'.")
            self.Valid = False
        self.Keys = [p for p in pathstring.split('/') if p != '']
        self.Counter = -1

    def getNextKey(self):
        if self.Counter < len(self.Keys) - 1:
            self.Counter += 1
            return self.Keys[self.Counter]
        else:
            raise ValueError("ERROR Not enough levels in path ! Can't reach farther than the last path key.")

    def isLast(self):
        return self.Counter == len(self.Keys) - 1

    def __repr__(self):
        return self.pathstring
