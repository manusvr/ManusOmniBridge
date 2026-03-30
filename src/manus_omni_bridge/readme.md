1. Run the Standard Hand (Defaults)
Since we set everything to false by default, running the basic launch command starts the standard hand with direct mapping and no spam in the terminal.
Bash

ros2 launch manus_omni_bridge manus_omni_bridge.launch.py

2. Run the Pro Hand
Bash

ros2 launch manus_omni_bridge manus_omni_bridge.launch.py is_pro:=true

3. Run the Pro Hand + Retargeting
Bash

ros2 launch manus_omni_bridge manus_omni_bridge.launch.py is_pro:=true manus_retargeting:=true

4. Debug Mode (Any configuration)
If something looks wrong with the fingers and you want to see the joint arrays printing in real-time, just tack on the debug argument:
Bash

ros2 launch manus_omni_bridge manus_omni_bridge.launch.py is_pro:=true debug_print