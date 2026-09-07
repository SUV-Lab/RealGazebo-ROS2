"""World wind relay: UDP MessageID=6 (UE -> manager) -> gz WindEffects.

ros_gz_bridge has no gz.msgs.Wind mapping, so the manager publishes on
gz-transport directly, to the topic the WindEffects world system
subscribes to (/world/<world>/wind). That system is loaded from PX4's
server.config and keeps the last command as world state, so a vehicle
spawned after the command feels the wind too - nothing is cached here.

Which links feel it: WindEffects pushes every link that declares
<enable_wind>true</enable_wind> with F = m * k * (wind - v), k being the
force_approximation_scaling_factor in PX4's server.config (tuned for an
x500-class airframe). That flag is opt-in ONLY for airframes without an
aerodynamic model (multirotors: the x500 family via x500_base). Airframes
with LiftDrag (fixed-wing / VTOL such as lc_62) must not set it: LiftDrag
already subtracts the world wind from the link velocity, and the extra
point-mass term would dominate them. Because LiftDrag reads the wind
entity's velocity even while WindEffects is disabled - and WindEffects
freezes that velocity mid-ramp when told enable_wind=false (it follows
commands through a ~1 s low-pass filter) - wind OFF is published as
enable_wind=true with a zero velocity, never as enable_wind=false.
"""


class WindPublisher:
    """One persistent gz-transport publisher for the world wind topic.

    Create it once on the main thread after the gz server is up (the
    manager does this right before the UDP listener starts); publishing
    from the UDP listener thread afterwards is fine. Advertising per
    command would race gz-transport discovery on every packet.
    """

    def __init__(self, world):
        # Imported lazily: the bindings ship with gz-harmonic in the sim
        # image, but the unit tests (and a host shell) do not have them.
        from gz.transport13 import AdvertiseMessageOptions, Node
        from gz.msgs10.wind_pb2 import Wind
        self._Wind = Wind
        self.topic = f'/world/{world}/wind'
        self._node = Node()  # must outlive the publisher
        self._pub = self._node.advertise(
            self.topic, Wind, AdvertiseMessageOptions())

    def has_subscriber(self) -> bool:
        """True once the WindEffects system is connected (discovery done)."""
        return self._pub.has_connections()

    def publish(self, enable, velocity) -> bool:
        msg = self._Wind()
        (msg.linear_velocity.x,
         msg.linear_velocity.y,
         msg.linear_velocity.z) = velocity
        # proto3 default is False: leaving it unset would switch wind OFF
        msg.enable_wind = bool(enable)
        return self._pub.publish(msg)
