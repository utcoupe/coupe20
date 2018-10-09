# Description : AIScheduler

This package takes care of choosing what actions to take based on an `actions` definition list.
The definition files are located under `src/Definitions`.

This package also contains the main launch files that start all system nodes.
In order to launch the main program with all system nodes, run `roslaunch robot_ai_scheduler main.launch` after sourcing the workspace.

## AI Scheduler

The scheduler is a fully-recursive task manager and executer. Calling the `findNext()` function in the `Strategy` class
will recursively call the ActionLists and Actions until an Order is returned. This Order is then triggered with its function
`execute()`, which will send an `action` message to the node specified in the XML definition file.

__NOTE 1__ : For now, the orders are implemented as services. They will soon be sent as `action` messages. This will enable the AI to execute several
Orders at the same time.

__NOTE 2__ : Conditions (e.g. ask for the Map server if there are enough elements in a container) are
yet to be implemented.

### Order definitions
The orders are defined in the file `3_orders.xml`. The `<order>` node has to have a `ref` attribute, in order for it to be referenced from the outside. It can take an optional `duration` attribute, which is a manual estimation. This node must have a `<message>` child node, with a `dest` attributes, representing the service or action the message will be sent to. A message node holds multiple `<param>`, mirroring the structure of the ROS request message sent. Each `<param>` has a `name`, matching the name of the parameter in the ROS message, and a `type`, used to parse the parameter. A parameter can be `optional`, which means it doesn't have to be filled when the order is called (the default value will be put in by ROS). It can be `preset` : in that case, the parameter cannot be set when the order is called, the value is constant and cannot be changed. To finish, default values can be set in the order definition. Note that a param with a default value is therefore optional, and a param cannot be preset and optional.

#### Example
Let's assume we want to send a request to `/asserv/goto`, with the following ROS definition :
```
geometry_msgs/Pose2D position
float64 number
uint8 command
string message
```

Here is a valid example of the order definition, with all the elements seen above :
```xml
<order ref="goto" duration="1">
  <message dest="/asserv/goto">
    <param name="position" type="pose2d" />                   <!-- regular parameter, required -->
    <param name="number" type="float">42.8</param>            <!-- parameter with a default value -->
    <param name="command" type="int" preset="true">5</param>  <!-- preset parameter -->
    <param name="message" type="string" optional="true"/>     <!-- optional parameter -->
  </message>
</order>
```

### Order references
The orders can be referenced from the actions or strategies definition files. To reference an order, a node `<orderref>` has to have its `ref` attribute matching the `ref` attribute of the referenced order. As childs of the node, the parameters (non-preset) are set, with the tag of the child matching the name of the parameter.

#### Example
Let's continue with our defined order from above. Suppose we want to reference it, here is a valid way to do it :
```xml
<orderref ref="goto">
  <position>
    <x>55.2</x>
    <y>57.1</y>
    <theta>3.14159</theta>
  </position>
  <message>hello world!</message> <!-- this node is optional -->
</orderref>
```

### Action definitions

The actions are defined in the file `2_actions.xml`. The `<action>` node has to have a `ref` attribute, in order for it to be referenced from the outside. This node has three children :
- The `<conditions>` node is yet to be implemented.
- The `<params>` node holds all the parameters used when calling this action. They are defined identically to the parameters in the message of an order, seen above, with the `name`, `type`, `preset` and `optional` attributes. For them to be useful, the parameters given to an action have to be passed to the orders or the actions that are called from this action. This will be discussed in the next part.
- The `<action>` node doc is TODO

### Action parameters binding
An action has a bunch of children (`orderref` and/or `actionref`). The parameters of the parent action can be passed to these children for them to use them. To do that, we use the `bind` attributes on the children's parameters like so :

```xml
<action ref="goto_spawn">
  <params>
    <param name="speed"/>
  </params>
  <actions exec="all" order="linear">
    <orderref ref="wheels_goto">
      <target_pos>
        <x>6.5</x>
        <y>7.5</y>
        <theta>7</theta>
      </target_pos>
      <speed bind="speed"/>
    </orderref>
  </actions>
</action>
```
Here, the order `wheels_goto` requires two parameters. The `target_pos` one is given directly in the `orderref`. However, no value is passed for the `speed` one. Instead, it is binded to the parameter named `speed` of the parent action. The `speed` parameter of this action is defined in the `params` node. When the `goto_spawn` action will be referenced, the `speed` parameter given to it will be passed to the order reference of `wheels_goto`. Note that the names for the parent parameter and the binded parameter do not have to match.

### Action references

The call of an action works the same as the order reference, replacing `orderref` by `actionref`.

### Adding new parameter types
In order to parse correctly all wanted type, one has to add a parser class, child of the `Param` class, for each type of parameter. Check out `ai_params.py` for more details.
