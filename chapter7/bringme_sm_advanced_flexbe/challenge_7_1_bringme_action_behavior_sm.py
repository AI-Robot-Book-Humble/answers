#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 Keith Valentin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Bringme Action Behavior.

Created on May 28, 2024
@author: Keith Valentin
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from bringme_sm_flexbe_states.manipulation_action_state import ManipulationActionState
from bringme_sm_flexbe_states.navigation_action_state import NavigationActionState
from bringme_sm_flexbe_states.vision_action_state import VisionActionState
from bringme_sm_flexbe_states.voice_action_state import VoiceActionState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class BringmeActionBehaviorSM(Behavior):
    """
    Define Bringme Action Behavior.

    Bringmeタスクのステートマシンの一例

    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Bringme Action Behavior'

        # parameters of this behavior
        self.add_parameter('listen_time', '10')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ManipulationActionState.initialize_ros(node)
        NavigationActionState.initialize_ros(node)
        VisionActionState.initialize_ros(node)
        VoiceActionState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        timeout = 15
        # x:946 y:136, x:565 y:360
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.time = self.listen_time

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:113 y:120
            OperatableStateMachine.add('Voice',
                                       VoiceActionState(timeout=timeout, action_topic="/speech_recognition/command"),
                                       transitions={'done': 'Navigation', 'failed': 'Voice', 'canceled': 'failed', 'timeout': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'canceled': Autonomy.Off, 'timeout': Autonomy.Off},
                                       remapping={'time': 'time', 'text': 'text', 'target': 'target', 'destination': 'destination'})

            # x:307 y:57
            OperatableStateMachine.add('Navigation',
                                       NavigationActionState(timeout=timeout, action_topic="ps_navigation/command"),
                                       transitions={'done': 'Vision', 'failed': 'Navigation', 'canceled': 'failed', 'timeout': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'canceled': Autonomy.Off, 'timeout': Autonomy.Off},
                                       remapping={'destination': 'destination', 'text': 'text'})

            # x:511 y:120
            OperatableStateMachine.add('Vision',
                                       VisionActionState(timeout=timeout, action_topic="ps_vision/command"),
                                       transitions={'done': 'Manipulation', 'failed': 'Manipulation', 'canceled': 'failed', 'timeout': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'canceled': Autonomy.Off, 'timeout': Autonomy.Off},
                                       remapping={'target': 'target', 'text': 'text'})

            # x:751 y:57
            OperatableStateMachine.add('Manipulation',
                                       ManipulationActionState(timeout=timeout, action_topic="ps_manipulation/command"),
                                       transitions={'done': 'finished', 'failed': 'Vision', 'canceled': 'failed', 'timeout': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'canceled': Autonomy.Off, 'timeout': Autonomy.Off},
                                       remapping={'target': 'target', 'text': 'text'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
