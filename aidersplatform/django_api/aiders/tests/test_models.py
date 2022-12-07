from aiders import models
from aiders.factories import (BuildMapSessionFactory, DetectionSessionFactory,
                              DroneFactory, ErrorMessageFactory,
                              MissionFactory, MissionLogFactory,
                              OperationFactory, TelemetryFactory)
from django.contrib.auth import get_user_model
from django.shortcuts import get_object_or_404

from .test_setup import TestSetUp


class TestModels(TestSetUp):

    def test_users_should_not_be_members_of_stopped_operations(self):
        operation_object = OperationFactory.create(operation_name=self.operation_name,
                                     operator=self.user)
        self.assertEquals(operation_object.active,True) #Operation is active once it is started
        qs = models.Operation.objects.filter(pk=operation_object.id)
        mod = get_object_or_404(qs)
        mod.active = False #Make operation inactive. That should remove all users from this operation
        mod.save()
        qs = get_user_model().objects.filter(pk=self.user.id)
        user_joined_op = get_object_or_404(qs).joined_operation
        self.assertEquals(user_joined_op, None)

    def test_user_automatically_joins_operation_once_he_creates_it(self):
        operation_object = OperationFactory.create(operation_name=self.operation_name,
                                     operator=self.user)
        qs = get_user_model().objects.filter(pk=self.user.id)
        user_joined_op = get_object_or_404(qs).joined_operation
        self.assertEquals(user_joined_op.id, operation_object.id)

    def test_Drone_save(self):
        operation_object = OperationFactory.create(operation_name=self.operation_name,operator=self.user)
        drone_object = DroneFactory.create(drone_name=self.drone_name, operation=operation_object)
        detection_object = models.Detection.objects.get(drone=drone_object)
        self.assertEquals(drone_object.id, detection_object.drone.id)

    def test_DetectionSession_save(self):
        operation_object = OperationFactory.create(operation_name=self.operation_name,operator=self.user)
        drone_object = DroneFactory.create(drone_name=self.drone_name, operation=operation_object)
        detection_object_1=DetectionSessionFactory.create(drone=drone_object, is_active=True)
        detection_object_2=DetectionSessionFactory.create(drone=drone_object, is_active=True)
        detection_object_1=models.DetectionSession.objects.get(id=detection_object_1.id)
        self.assertEquals(detection_object_1.is_active, False)

    def test_ErrorMessage_save(self):
        operation_object = OperationFactory.create(operation_name=self.operation_name,operator=self.user)
        drone_object = DroneFactory.create(drone_name=self.drone_name, operation=operation_object)
        error_message= ErrorMessageFactory.create(drone=drone_object)
        self.assertEquals(drone_object.operation, error_message.operation)
    
    def test_Telemetry_save(self):
        operation_object = OperationFactory.create(operation_name=self.operation_name,operator=self.user)
        drone_object = DroneFactory.create(drone_name=self.drone_name, operation=operation_object)
        mission_object=MissionFactory.create(operation=operation_object, user=self.user)
        MissionLogFactory.create(action="START_MISSION", mission=mission_object, operation=operation_object, user=self.user, drone=drone_object)
        TelemetryFactory.create(drone=drone_object, drone_state='In_Mission')
        TelemetryFactory.create(drone=drone_object, drone_state='Flying')
        finished_mission_log=models.MissionLog.objects.filter(operation=operation_object, user=self.user, drone=drone_object).last()
        if finished_mission_log.action=='FINISH_MISSION':
            assert(True)
        else:
            assert(False)
