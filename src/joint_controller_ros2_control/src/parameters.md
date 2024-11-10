# Joint Controller Parameters

Default Config
```yaml
joint_controller:
  ros__parameters:
    command_interfaces: effort
    joint_names: '{}'
    joint_params:
      <joint_names>:
        effort_max: 0.0
        position_max: 0.0
        position_min: 0.0
        velocity_max: 0.0
    pid_gains:
      <joint_names>:
        d: 0.0
        i: 0.0
        ilimit: 0.0
        p: 0.0
    state_interfaces: '{position, velocity}'

```

## joint_names

Specifies joint_names or axes used by the controller.

* Type: `string_array`
* Default Value: {}
* Read only: True

*Constraints:*
 - contains no duplicates
 - parameter is not empty

*Additional Constraints:*



## command_interfaces

Name of the interfaces used by the controller for writing commands to the hardware.

* Type: `string`
* Default Value: "effort"
* Read only: True

*Constraints:*
 - parameter is not empty
 - length is greater than 0
 - length is less than 1
 - one of the specified values: effort

*Additional Constraints:*



## state_interfaces

Name of the interfaces used by the controller getting hardware states and reference commands.

* Type: `string_array`
* Default Value: {"position", "velocity"}
* Read only: True

*Constraints:*
 - parameter is not empty
 - length is greater than 0
 - length is less than 3
 - every element is one of the list position

*Additional Constraints:*



## joint_params.<joint_names>.position_max

Proportional gain for PID

* Type: `double`
* Default Value: 0.0

## joint_params.<joint_names>.position_min

Integral gain for PID

* Type: `double`
* Default Value: 0.0

## joint_params.<joint_names>.velocity_max

Derivative gain for PID

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## joint_params.<joint_names>.effort_max

Integrator limit.

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## pid_gains.<joint_names>.p

Proportional gain for PID

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*



## pid_gains.<joint_names>.i

Integral gain for PID

* Type: `double`
* Default Value: 0.0

## pid_gains.<joint_names>.d

Derivative gain for PID

* Type: `double`
* Default Value: 0.0

## pid_gains.<joint_names>.ilimit

Integrator limit for PID.

* Type: `double`
* Default Value: 0.0

*Constraints:*
 - greater than or equal to 0.0

*Additional Constraints:*


