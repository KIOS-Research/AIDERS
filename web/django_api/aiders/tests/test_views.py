from aiders import models, views
from aiders.factories import (AlgorithmFactory, BuildMapImageFactory1,
                              BuildMapSessionFactory, DroneFactory,
                              MissionFactory, MissionLogFactory, MissionPointFactory,
                              OperationFactory, TelemetryFactory)
from django.contrib.auth.models import Group
from django.urls import reverse
from formtools.wizard.views import WizardView
from guardian.shortcuts import assign_perm
from rest_framework import status

from .test_setup import TestSetUp

# pdb.set_trace()


class TestViews(TestSetUp):
    # Does not work Fix later
    # def test_DatabaseFiller_view_load(self):
    #     self.client.force_login(self.user)
    #     response = self.client.get(reverse('fill_dummy_data'), follow=True)
    #     self.assertEqual(response.status_code, status.HTTP_200_OK)
    #     self.assertTemplateUsed(response, 'base.html','aiders/login.html')
    # def test_DatabaseFiller_view_deny_anonymous(self):
    #     response = self.client.get(reverse('fill_dummy_data'), follow=True)
    #     self.assertRedirects(response, reverse('login')+'?next='+reverse('fill_dummy_data'))

    def test_OperationListCreateAPIView_view(self):
        OperationFactory.create(operation_name=self.operation_name,
                                operator=self.user)  # Create an operation and insert it in DB
        list_create_op_view = views.OperationListCreateAPIView.as_view()
        request = self.factory.get(reverse('operations'),
                                   format='json')  # Get request to perform the operation we just retrieved
        request.user = self.user
        response = list_create_op_view(request)
        retrieved_operation_name = response.data[0].get('operation_name')
        self.assertEqual(models.Operation.objects.all().first().operation_name,
                         retrieved_operation_name)  # Check if the operation we retrieve is the same as the one we inserted on the database

    def test_OperationListCreateAPIView_view_load(self):
        self.client.force_login(self.user)
        response = self.client.get(reverse('operations'))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_OperationListCreateAPIView_view_deny_anonymous(self):
        response = self.client.get(reverse('operations'), follow=True)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('operations'))

    def test_DroneListCreateAPIView_insert_drone(self):
        drones_view = views.DroneListCreateAPIView.as_view()
        prev_drone_count = models.Drone.objects.all().count()
        op = OperationFactory.create(operation_name=self.operation_name,
                                     operator=self.user)  # Before inserting a drone, we first have to insert an operation so I can assign the drone into that
        drones_url = reverse(
            'drones', kwargs={'operation_name': op.operation_name})
        post_data = {'drone_name': self.drone_name,
                     'model': 'test_model',
                     'camera_model': 'test_camera_model',
                     'operation': op.id
                     }

        request = self.factory.post(drones_url, post_data)
        request.user = self.user
        response = drones_view(request)

        self.assertEquals(response.status_code, status.HTTP_201_CREATED)
        self.assertEquals(models.Drone.objects.all().count(),
                          prev_drone_count + 1)
        self.assertEquals(models.Drone.objects.all(
        ).first().drone_name, self.drone_name)

    def test_DroneListCreateAPIView_view_load(self):
        self.client.force_login(self.user)
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('drones', args=[self.operation_name]), follow=True)
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_DroneListCreateAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('drones', args=[self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('drones', args=[self.operation_name]))

    def test_DroneRetrieveAPIView_view_post(self):
        drones_view = views.DroneRetrieveAPIView.as_view()
        op = OperationFactory.create(operation_name=self.operation_name,
                                     operator=self.user)  # Before inserting a drone, we first have to insert an operation so I can assign the drone into that
        DroneFactory.create(drone_name=self.drone_name, operation=op)
        drones_url = reverse('drone', kwargs={
                             'operation_name': op.operation_name, 'drone_name': self.drone_name})
        request = self.factory.get(drones_url, format='json')
        request.user = self.user
        response = drones_view(
            request, drone_name=self.drone_name, operation_name=op.operation_name)
        self.assertEquals(response.status_code, status.HTTP_200_OK)
        retrieved_drone_name = response.data.get('drone_name')
        self.assertEquals(retrieved_drone_name, self.drone_name)

    def test_DroneRetrieveAPIView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        response = self.client.get(
            reverse('drone', args=[self.operation_name, self.drone_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_DroneRetrieveAPIView_view_deny_anonymous(self):
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        response = self.client.get(
            reverse('drone', args=[self.operation_name, self.drone_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('drone', args=[self.operation_name, self.drone_name]))

    def test_DetectionRetrieveAPIView_view_stats_for_a_drone(self):
        """
        Test that the detection stats are returned for a drone
        :return:
        """
        detection_view = views.DetectionRetrieveAPIView.as_view()
        op = OperationFactory.create(operation_name=self.operation_name,
                                     operator=self.user)  # Before inserting a drone, we first have to insert an operation so I can assign the drone into that
        drone = DroneFactory.create(drone_name=self.drone_name, operation=op)

        detection_url = reverse('detection', kwargs={
                                'operation_name': op.operation_name, 'drone_name': drone.drone_name})
        # post_data = {'drone_name': self.drone_name,
        #              'model': 'test_model',
        #              'operation': op.id
        # }
        request = self.factory.get(detection_url, format='json')
        request.user = self.user

        response = detection_view(
            request, drone_name=drone.drone_name, operation_name=op.operation_name)

        self.assertEquals(models.Detection.objects.get(
            drone=drone).id, response.data.get('id'))

    def test_MissionLoggerListCreateAPIView_view_load(self):
        self.client.force_login(self.user)
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('missions_logger', args=[self.operation_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_MissionLoggerListCreateAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('missions_logger', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('missions_logger', args=[self.operation_name]))

    def test_MissionRetrieveAPIView_view_success(self):
        """
        Given a drone name, get on which mission it currently participates
        Returns:

        """
        mission_view = views.MissionRetrieveAPIView.as_view()

        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        group_join_operation = Group.objects.create(
            name=operation_instance.operation_name+" operation join")
        assign_perm('join_operation', group_join_operation, operation_instance)
        self.user.groups.add(group_join_operation)
        mission = MissionFactory.create(
            mission_type='NORMAL_MISSION', user=self.user, operation=operation_instance)
        drone = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance, mission=mission)

        mission_url = reverse('mission', kwargs={
                              'operation_name': operation_instance.operation_name, 'drone_name': self.drone_name})
        request = self.factory.get(mission_url, format='json')
        request.user = self.user
        response = mission_view(
            request, operation_name=operation_instance.operation_name,  drone_name=drone.drone_name)

        self.assertEquals(response.status_code, status.HTTP_200_OK)
        self.assertEquals(models.Mission.objects.all().first().id, mission.id)

    def test_MissionRetrieveAPIView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        group_join_operation = Group.objects.create(
            name=operation_instance.operation_name+" operation join")
        assign_perm('join_operation', group_join_operation, operation_instance)
        self.user.groups.add(group_join_operation)
        response = self.client.get(
            reverse('replay_mission', args=[self.operation_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_MissionRetrieveAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('replay_mission', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('replay_mission', args=[self.operation_name]))

    def test_ReplayMissionOnlineAPIView_view_load(self):
        self.client.force_login(self.user)
        operation_object = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_object = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_object)
        mission_object = MissionFactory.create(
            operation=operation_object, user=self.user)
        MissionLogFactory.create(action="START_MISSION", mission=mission_object,
                                 operation=operation_object, user=self.user, drone=drone_object)
        MissionLogFactory.create(action="FINISH_MISSION", mission=mission_object,
                                 operation=operation_object, user=self.user, drone=drone_object)
        response = self.client.get(reverse('replay_mission_engine', args=[
                                   self.operation_name, mission_object.id]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_ReplayMissionOnlineAPIView_view_deny_anonymous(self):
        operation_object = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_object = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_object)
        mission_object = MissionFactory.create(
            operation=operation_object, user=self.user)
        MissionLogFactory.create(action="START_MISSION", mission=mission_object,
                                 operation=operation_object, user=self.user, drone=drone_object)
        MissionLogFactory.create(action="FINISH_MISSION", mission=mission_object,
                                 operation=operation_object, user=self.user, drone=drone_object)
        response = self.client.get(reverse('replay_mission_engine', args=[
                                   self.operation_name, mission_object.id]))
        self.assertRedirects(response, reverse('login')+'?next='+reverse(
            'replay_mission_engine', args=[self.operation_name, mission_object.id]))

    def test_TelemetryListCreateAPIView_view_insert_telemetry(self):
        prev_telemetry_count = models.Telemetry.objects.all().count()
        telemetry_view = views.TelemetryListCreateAPIView.as_view()

        op = OperationFactory.create(operation_name=self.operation_name,
                                     operator=self.user)
        drone = DroneFactory.create(drone_name=self.drone_name, operation=op)

        telemetry_url = reverse('telemetries', kwargs={
                                'operation_name': op.operation_name})
        post_data = {
            'drone': drone.id,
            'battery_percentage': 20.4,
            'gps_signal': 0.5,
            'satellites': 14,
            'heading': 40.5,
            'velocity': 4.3,
            'homeLat': 35.43,
            'homeLon': 32.34,
            'lat': 34.53,
            'lon': 35.32,
            'alt': 30.4,
            'drone_state': 'Flying',
            'secondsOn': 43.3,
            'gimbal_angle': 30,
        }
        request = self.factory.post(telemetry_url, post_data)
        request.user = self.user
        response = telemetry_view(request)

        inserted_telemetry_id = response.data['id']

        self.assertEquals(response.status_code, status.HTTP_201_CREATED)
        self.assertEquals(models.Telemetry.objects.all().count(),
                          prev_telemetry_count + 1)
        self.assertEquals(
            models.Telemetry.objects.all().first().id, inserted_telemetry_id)

    def test_TelemetryListCreateAPIView_view_load(self):
        self.client.force_login(self.user)
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('telemetries', args=[self.operation_name]), follow=True)
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_TelemetryListCreateAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('telemetries', args=[self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('telemetries', args=[self.operation_name]))

    def test_TelemetryRetrieveAPIView_view(self):
        telemetry_view = views.TelemetryRetrieveAPIView.as_view()
        op = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone = DroneFactory.create(drone_name=self.drone_name, operation=op)
        telemetry = TelemetryFactory.create(drone=drone)
        telemetry_url = reverse('telemetry', kwargs={
                                'drone_name': drone.drone_name, 'operation_name': op.operation_name})
        request = self.factory.get(telemetry_url, format='json')
        request.user = self.user
        response = telemetry_view(
            request, drone_name=drone.drone_name, operation_name=op.operation_name)
        self.assertEquals(response.status_code, status.HTTP_200_OK)
        self.assertEquals(
            models.Telemetry.objects.all().first().id, telemetry.id)

    def test_TelemetryRetrieveAPIView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        TelemetryFactory.create(drone=drone_instance)
        response = self.client.get(
            reverse('telemetry', args=[self.operation_name, self.drone_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_TelemetryRetrieveAPIView_view_deny_anonymous(self):
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        response = self.client.get(
            reverse('telemetry', args=[self.operation_name, self.drone_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('telemetry', args=[self.operation_name, self.drone_name]))

    def test_MissionPointsListCreateAPIView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        mission_instance = MissionFactory.create(
            mission_type="NORMAL_MISSION", operation=operation_instance, user=self.user)
        mission_points_instance = MissionPointFactory.create_batch(1)
        mission_instance.mission_points.set(mission_points_instance)
        mission_instance.save()
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance, mission=mission_instance)
        response = self.client.get(reverse('mission_points', args=[
                                   self.operation_name, self.drone_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_MissionPointsListCreateAPIView_view_load_with_out_a_mission(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        response = self.client.get(reverse('mission_points', args=[
                                   self.operation_name, self.drone_name]))
        self.assertEqual(response.status_code, status.HTTP_404_NOT_FOUND)

    def test_MissionPointsListCreateAPIView_view_deny_anonymous(self):
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        mission_instance = MissionFactory.create(
            mission_type="NORMAL_MISSION", operation=operation_instance, user=self.user)
        mission_points_instance = MissionPointFactory.create_batch(1)
        mission_instance.mission_points.set(mission_points_instance)
        mission_instance.save()
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance, mission=mission_instance)
        response = self.client.get(reverse('mission_points', args=[
                                   self.operation_name, self.drone_name]))
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('mission_points', args=[self.operation_name, self.drone_name]))

    def test_UserList_view_load(self):
        self.client.force_login(self.user)
        response = self.client.get(reverse('users'))
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertTemplateUsed(response, 'base.html', 'aiders/users.html')

    def test_UserList_view_deny_anonymous(self):
        response = self.client.get(reverse('users'), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('users'))

    def test_DroneList_view_load(self):
        self.client.force_login(self.user)
        response = self.client.get(reverse('drones_list'))
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertTemplateUsed(response, 'base.html', 'aiders/drones.html')

    def test_DroneList_view_deny_anonymous(self):
        response = self.client.get(reverse('drones_list'), follow=True)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('drones_list'))

    def test_UserDetail_detail_view_load(self):
        self.client.force_login(self.user)
        response = self.client.get(reverse('user_detail', args=[1]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_UserDetail_view_deny_anonymous(self):
        response = self.client.get(
            reverse('user_detail', args=[1]), follow=True)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('user_detail', args=[1]))

    def test_AlgorithmRetrieveView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        algorithm_instance = AlgorithmFactory.create(
            operation=operation_instance, user=self.user)
        response = self.client.get(reverse('algorithm_result', args=[
                                   self.operation_name, algorithm_instance.pk, "input"]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_AlgorithmRetrieveView_view_deny_anonymous(self):
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        algorithm_instance = AlgorithmFactory.create(
            operation=operation_instance)
        response = self.client.get(reverse('algorithm_result', args=[
                                   self.operation_name, algorithm_instance.pk, "input"]), follow=True)
        self.assertRedirects(response, reverse('login')+'?next='+reverse(
            'algorithm_result', args=[self.operation_name, algorithm_instance.pk, "input"]))

    def test_ManageOperationsView_view_load(self):
        self.client.force_login(self.user)
        response = self.client.get(reverse('manage_operations'))
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertTemplateUsed(response, 'base.html',
                                'aiders/manage_operations.html')

    def test_ManageOperationsView_view_deny_anonymous(self):
        response = self.client.get(reverse('manage_operations'), follow=True)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('manage_operations'))

    def test_ManagePermissionsView_view_load(self):
        self.client.force_login(self.user)
        models.check_groups()
        group = Group.objects.get(name='edit_permissions')
        self.user.groups.add(group)
        response = self.client.get(reverse('manage_permissions'))
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertTemplateUsed(response, 'base.html',
                                'aiders/manage_permissions.html')

    def test_ManagePermissionsView_view_post_success(self):
        self.client.force_login(self.user)
        models.check_groups()
        group = Group.objects.get(name='edit_permissions')
        self.user.groups.add(group)
        response = self.client.post(reverse('manage_permissions'), {
                                    'permission_edit_permissions': self.user.pk, 'permission_create_operations': self.user.pk})
        self.assertEqual(response.status_code, status.HTTP_202_ACCEPTED)
        self.assertTemplateUsed(response, 'base.html',
                                'aiders/manage_permissions.html')
        self.assertEqual(len(self.user.groups.all()), 3)

    def test_ManagePermissionsView_view_deny_anonymous(self):
        response = self.client.get(reverse('manage_permissions'), follow=True)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('manage_permissions'))

    def test_DroneModifyOperationView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=None)
        operation_instance.drones_to_operate.add(drone_instance)
        response = self.client.get(
            reverse('drone_modify_operation', args=[drone_instance.drone_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_DroneModifyOperationView_view_post_success(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=None)
        operation_instance.drones_to_operate.add(drone_instance)
        post_data = {
            'operation_name': operation_instance.operation_name
        }
        response = self.client.post(reverse('drone_modify_operation', args=[
                                    drone_instance.drone_name]), post_data)
        self.assertEquals(response.status_code, status.HTTP_202_ACCEPTED)
        self.assertEquals(models.Drone.objects.filter(
            drone_name=self.drone_name).last().operation, operation_instance)

    def test_DroneModifyOperationView_view_post_fail(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=None)
        operation_instance.drones_to_operate.add(drone_instance)
        post_data = {
            'operation_name': "random_operation_name"
        }
        response = self.client.post(reverse('drone_modify_operation', args=[
                                    drone_instance.drone_name]), post_data)
        self.assertEquals(response.status_code, status.HTTP_400_BAD_REQUEST)
        self.assertEquals(models.Drone.objects.filter(
            drone_name=self.drone_name).last().operation, None)

    def test_DroneModifyOperationView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(drone_name=self.drone_name)
        response = self.client.get(reverse('drone_modify_operation', args=[
                                   drone_instance.drone_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('drone_modify_operation', args=[drone_instance.drone_name]))

    def test_BuildMapAPIView_view_post_start_success(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        TelemetryFactory.create(drone=drone_instance)
        response = self.client.post(reverse('start_build_map', args=[self.operation_name]), {
                                    'drone_id': self.drone_name, 'start_build_map_boolean': 'true', 'start_multispectral_build_map': False, 'overlap': 0})
        self.assertEquals(response.status_code, status.HTTP_202_ACCEPTED)

    def test_BuildMapAPIView_view_post_stop_success(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        response = self.client.post(reverse('start_build_map', args=[self.operation_name]), {
                                    'drone_id': self.drone_name, 'start_build_map_boolean': 'false', 'start_multispectral_build_map': False, 'overlap': 0})
        self.assertEquals(response.status_code, status.HTTP_202_ACCEPTED)

    def test_BuildMapAPIView_view_post_fail(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        telemetry_instance = TelemetryFactory.create(drone=drone_instance)
        response = self.client.post(reverse('start_build_map', args=[self.operation_name]), {
                                    'drone_id': self.drone_name, 'random_data': 'random_data', 'random_data': False})
        self.assertEquals(response.status_code, status.HTTP_400_BAD_REQUEST)

    def test_BuildMapAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('start_build_map', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('start_build_map', args=[self.operation_name]))

    def test_BuildMapLoadAPIView_view_load(self):
        self.client.force_login(self.user)
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('load_build_map', args=[self.operation_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_BuildMapLoadAPIView_view_post_success(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        build_map_session_instance = BuildMapSessionFactory.create(
            user=self.user, operation=operation_instance, drone=drone_instance)
        post_data = {
            'build_map_id': build_map_session_instance.pk
        }
        response = self.client.post(reverse('load_build_map', args=[
                                    self.operation_name]), post_data, format='json')
        self.assertEqual(response.status_code, status.HTTP_201_CREATED)

    def test_BuildMapLoadAPIView_view_post_fail(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        post_data = {
            'random_data': 'random_data'
        }
        response = self.client.post(reverse('load_build_map', args=[
                                    self.operation_name]), post_data, format='json')
        self.assertEqual(response.status_code, status.HTTP_400_BAD_REQUEST)

    def test_BuildMapLoadAPIView_post_fail_2(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        post_data = {
            'build_map_id': 1
        }
        response = self.client.post(reverse('load_build_map', args=[
                                    self.operation_name]), post_data, format='json')
        self.assertEqual(response.status_code, status.HTTP_400_BAD_REQUEST)

    def test_BuildMapLoadAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('load_build_map', args=[self.operation_name]))
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('load_build_map', args=[self.operation_name]))

    def test_FirePredictionCreateAPIView_view_post_success(self):
        self.client.force_login(self.user)
        prev_algorithm_count = models.Algorithm.objects.all().count()
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        post_data = {
            'fire_speed': 10,
            'fire_fronts': 10,
            "wind_speed": 10,
            "wind_angle": 10,
            "time_steps": {
                "value": 100,
                "units": 'seconds'
            },
            "location": {
                "lon": 35,
                "lat": 33
            },
            "time_intervals": [10, 20, 30, 40, 50, 60, 70, 80, 100],
            "user": self.user.username,
        }
        response = self.client.post(reverse('fire_prediction', args=[
                                    self.operation_name]), post_data, format='json')
        created_algorithm_id = models.Algorithm.objects.latest('id').id
        self.assertEquals(response.status_code, status.HTTP_201_CREATED)
        self.assertEquals(models.Algorithm.objects.all().count(),
                          prev_algorithm_count + 1)
        self.assertEquals(
            models.Algorithm.objects.all().first().id, created_algorithm_id)

    def test_FirePredictionCreateAPIView_view_post_fail(self):
        self.client.force_login(self.user)
        prev_algorithm_count = models.Algorithm.objects.all().count()
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        post_data = {
            'random_data': 'random_data',
            'fire_fronts': 10,
            "wind_speed": 20000000,
            "location": {
                "lon": 35,
                "lat": 33
            },
            "time_intervals": 0,
            "user": self.user.username,
        }
        response = self.client.post(reverse('fire_prediction', args=[
                                    self.operation_name]), post_data, format='json')
        # created_algorithm_id = models.Algorithm.objects.latest('id').id
        self.assertEquals(response.status_code, status.HTTP_400_BAD_REQUEST)
        self.assertEquals(
            models.Algorithm.objects.all().count(), prev_algorithm_count)

    def test_FirePredictionCreateAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('fire_prediction', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('fire_prediction', args=[self.operation_name]))

    """CREATE login_view logout_view"""

    def test_login_view_load(self):
        response = self.client.get(reverse('login'))
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_login_view_post_fail(self):
        response = self.client.post(
            reverse('login'), {'username': 'random user', 'password': 'random password'})
        self.assertEquals(response.status_code, status.HTTP_404_NOT_FOUND)

    def test_ExecuteAlgorithmAPIView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('algorithm_execute', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('algorithm_execute', args=[self.operation_name]))

    def test_ExecuteMissionAPIView_view_deny_anonymous(self):
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        drone_instance = DroneFactory.create(
            drone_name=self.drone_name, operation=operation_instance)
        response = self.client.get(
            reverse('mission', args=[self.operation_name, self.drone_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('mission', args=[self.operation_name, self.drone_name]))

    def test_AlgorithmListView_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        group_join_operation = Group.objects.create(
            name=operation_instance.operation_name+" operation join")
        assign_perm('join_operation', group_join_operation, operation_instance)
        self.user.groups.add(group_join_operation)
        response = self.client.get(
            reverse('algorithms', args=[self.operation_name]))
        self.assertEqual(response.status_code, status.HTTP_200_OK)
        self.assertTemplateUsed(response, 'base.html',
                                'aiders/algorithms.html')

    def test_AlgorithmListView_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('algorithms', args=[self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('algorithms', args=[self.operation_name]))

    def test_stop_operation_view_view_load(self):
        self.client.force_login(self.user)
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(
            reverse('stop_operation', args=[self.operation_name]))
        self.assertEqual(response.status_code, 302)

    def test_stop_operation_view_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('stop_operation', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('stop_operation', args=[self.operation_name]))

    def test_leave_operation_view_view_load(self):
        self.client.force_login(self.user)
        response = self.client.get(reverse('leave_operation'))
        self.assertEqual(response.status_code, 302)

    def test_leave_operation_view_view_deny_anonymous(self):
        response = self.client.get(reverse('leave_operation'), follow=True)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('leave_operation'))

    def test_join_operation_view_view_load(self):
        self.client.force_login(self.user)
        operation_instance = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        group_join_operation = Group.objects.create(
            name=operation_instance.operation_name+" operation join")
        assign_perm('join_operation', group_join_operation, operation_instance)
        self.user.groups.add(group_join_operation)
        response = self.client.get(reverse('join_operation', args=[
                                   self.operation_name]), follow=True)
        response = self.client.post(reverse('join_operation', args=[self.operation_name, ]), {
                                    'operation_name': self.operation_name, }, follow=True)
        self.assertEqual(response.status_code, status.HTTP_200_OK)

    def test_join_operation_view_deny_anonymous(self):
        OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        response = self.client.get(reverse('join_operation', args=[
                                   self.operation_name]), follow=True)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('join_operation', args=[self.operation_name]))

    """CREATE register_request"""

    """WeatherStationAPIView"""


class TestWizard(WizardView):
    storage_name = 'formtools.wizard.storage.session.SessionStorage'

    def dispatch(self, request, *args, **kwargs):
        response = super().dispatch(request, *args, **kwargs)
        return response, self

    def get_form_kwargs(self, step, *args, **kwargs):
        kwargs = super().get_form_kwargs(step, *args, **kwargs)
        if step == 'kwargs_test':
            kwargs['test'] = True
        return kwargs


class TestWizard(TestSetUp):

    def test_wizard_operation_form_create(self):
        self.client.force_login(self.user)
        url = reverse('new_operation')

        models.check_groups()
        Group.objects.get(name='create_operations')
        self.user.groups.add(Group.objects.get(name='create_operations'))

        response = self.client.get(url)
        self.assertEqual(status.HTTP_200_OK, response.status_code)

    def test_wizard_operation_form_create_deny_anonymous(self):
        url = reverse('new_operation')

        models.check_groups()
        Group.objects.get(name='create_operations')
        self.user.groups.add(Group.objects.get(name='create_operations'))

        response = self.client.get(url)
        self.assertRedirects(response, reverse('login') +
                             '?next='+reverse('new_operation'))

    def test_wizard_operation_form_edit(self):
        self.client.force_login(self.user)

        operation = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        group_edit_operation = Group.objects.create(
            name=self.operation_name+" operation edit")
        assign_perm('edit_operation', group_edit_operation, operation)
        self.user.groups.add(group_edit_operation)

        url = reverse('edit_operation', args=(self.operation_name,))

        response = self.client.get(url)
        self.assertEqual(status.HTTP_200_OK, response.status_code)

    def test_wizard_operation_form_edit_deny_anonymous(self):
        operation = OperationFactory.create(
            operation_name=self.operation_name, operator=self.user)
        group_edit_operation = Group.objects.create(
            name=self.operation_name+" operation edit")
        assign_perm('edit_operation', group_edit_operation, operation)
        self.user.groups.add(group_edit_operation)

        url = reverse('edit_operation', args=(self.operation_name,))
        response = self.client.get(url)
        self.assertRedirects(response, reverse(
            'login')+'?next='+reverse('edit_operation', args=(self.operation_name,)))
