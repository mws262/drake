# Note that this script runs in the main context of drake-visulizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import numpy as np
import drake as lcmdrakemsg


class ArrowVisualizer(object):

    def __init__(self):
        self._folder_name = 'Marker Arrows'
        self._name = "Arrow Visualizer"
        self._enabled = False
        self._sub = None

        self.set_enabled(True)

    def add_subscriber(self):
        if self._sub is not None:
            return

        self._sub = lcmUtils.addSubscriber(
            'MARKER_ARROWS',
            messageClass=lcmdrakemsg.lcmt_arbitrary_arrow_collection,
            callback=self.handle_message)
        print self._name + " subscriber added."

    def remove_subscriber(self):
        if self._sub is None:
            return

        lcmUtils.removeSubscriber(self._sub)
        self._sub = None
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        print self._name + " subscriber removed."

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable):
        self._enabled = enable
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def handle_message(self, msg):
        # Limits the rate of message handling, since redrawing is done in the
        # message handler.
        self._sub.setSpeedLimit(0.1)

        # Removes the folder completely.
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))

        # Recreates folder.
        folder = om.getOrCreateContainer(self._folder_name)

        # A map from pair of body names to a list of contact forces
        collision_pair_to_forces = {}
        for arrow in msg.arrow_info:
            point = np.array([arrow.arrow_origin[0],
                              arrow.arrow_origin[1],
                              arrow.arrow_origin[2]])
            vec = np.array([arrow.arrow_vector[0],
                            arrow.arrow_vector[1],
                            arrow.arrow_vector[2]])

            d = DebugData()
            d.addArrow(start=point,
                       end=vec + point,
                       tubeRadius=0.005,
                       headRadius=0.01)

            vis.showPolyData(
                d.getPolyData(), str(0), parent=folder, color=[arrow.rgb[0], arrow.rgb[1], arrow.rgb[2]])


def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = ArrowVisualizer()

    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Creates the visualizer when this script is executed.
arrow_vis = init_visualizer()
