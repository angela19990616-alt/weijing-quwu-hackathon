# B70 TeleAvatar model client patch

This directory is a local snapshot of the client currently used on B70. It adds an opt-in right-arm-only mode:

```text
--freeze-left-arm
```

When enabled, the client captures the measured left seven-joint pose before the first action, clamps that snapshot to
the existing `arm_config.yml` software limits, and republishes the same target while the right arm remains controlled
by the policy. Policy output for the left gripper is not published, so its existing target is retained.

The option is off by default. Do not deploy or start this client on the physical robot without an onsite safety check.
