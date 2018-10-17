# AI Actions definition files

(_TODO writing in progress_)
This is where the robot is told what to do.

## 1. Files
- `1_strategies.xml`

File basic structure :
```xml
<strategies>
	<strategy name="utcoupe_ftw">
	... actionrefs and orderrefs
	</strategy>
	<strategy name="hithere">
	... actionrefs and orderrefs
	</strategy>
</strategies>
```

- `2_actions.xml`

File basic structure :
```xml
<actions>
	<action name="sth", ref="actuator_open">
	... actionrefs and orderrefs
	</action>
	<action name="sth", ref="actuator_close">
	... actionrefs and orderrefs
	</action>
</actions>
```




## 2. AI Objects

### 1. Strategies
The `1_Strategies.json` file is a JSON dictionary of several Strategy objects. When the whole system is launched, the user selects which strategy the robot has to follow.
This lets the user create several ways ("strategies") of achieving the final objective, and quickly execute the one he wants at each execution time.


### 2. ActionLists
Group tags are a list or actions. This list can be configured as :
- `linear` : Sorted (the robot follows the order of actions in the list given in the file)
- `mostreward` : The AI automatically picks the sequence that will give the most amount of reward points.
- `fastest`, : The AI automatically picks the sequence that will execute the quickest.
- `simultaneous`, fastest actions first.

When a group is executed, the conditions to consider it is successful or not can be configured :
- `all` : All actions must be done correctly.
- `one` : At least one action must be done correctly.
- `+` : Do all actions if possible, but if one is done it will be considered successful.
- `last` : The last action must be executed correctly.

Properties :
- `reward`: If the whole group execution is considered successful, this is the amount of reward points given.

### 3. Actions
Actions are almost the same as ActionLists, except they are necessarily defined in the file `2_actions.xml` and can be referenced by other actions.
An Action object is a group of `Orders`, `Actions` and `ActionLists`.


### 4. Orders
Properties :
- `time`, obligatory : Approximate time needed to complete an action (seconds).
- `reward` : If there are any, the action gives reward points to the AI.

Supported embedded actions that can be used in actions:
- The `wheels` group communicates with the ROS `robot_movement_wheels` package. This is how the AI asks for the robot to move from a point to another. Actions defined :
	- `wheels_gotoxy`: Ask the robot to move to a certain position. angle doesn't matter.
	- `wheels_gotoxya`: Go to a certain position and angle.
	- `wheels_gotoa`: Ask the robot to turn toward a certain direction.
	- `wheels_delay`: Stop the robot for a certain duration (seconds).
- The `actuators` group:
	- `actuator_open`: Set the actuator to the `closed` position set in the `robot.json` description file.
	- `actuator_close`: Set the actuator to the `open` position set in the `robot.json` description file.
	- `actuator_toggle`: Toggle between open/close position.
	- `actuator_setpos`: Manually set a position to the actuator.
