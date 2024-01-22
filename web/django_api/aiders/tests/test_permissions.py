from aiders import models
from aiders.factories import OperationFactory
from django.contrib.auth.models import Group
from guardian.shortcuts import assign_perm

from .test_setup import TestSetUp


class TestViews(TestSetUp):

    def test_user_doesnt_have_permission_for_edit_permissions(self):
        self.assertFalse(self.user.has_perm('edit_permissions'))

    def test_user_have_permission_for_edit_permissions(self):
        models.check_groups()
        group=Group.objects.get(name='edit_permissions')
        self.user.groups.add(group)
        self.assertTrue(self.user.has_perm('aiders.edit_permissions'))
    
    def test_user_doesnt_have_permission_for_create_operations(self):
        self.assertFalse(self.user.has_perm('aiders.create_operations'))

    def test_user_have_permission_for_create_operations(self):
        models.check_groups()
        Group.objects.get(name='create_operations')
        self.user.groups.add(Group.objects.get(name='create_operations'))
        self.assertTrue(self.user.has_perm('aiders.create_operations'))

    def test_user_doesnt_have_permission_for_join_operations(self):
        self.assertFalse(self.user.has_perm('aiders.join_operations'))

    def test_user_have_permission_for_join_operations(self):
        models.check_groups()
        Group.objects.get(name='join_operations')
        self.user.groups.add(Group.objects.get(name='join_operations'))
        self.assertTrue(self.user.has_perm('aiders.join_operations'))

    def test_user_doesnt_have_permission_for_specific_operation(self):
        operation=OperationFactory.create(operation_name=self.operation_name, operator=self.user)
        self.assertFalse(self.user.has_perm('join_operation', operation))

    def test_user_have_permission_for_specific_operation(self):
        operation=OperationFactory.create(operation_name=self.operation_name, operator=self.user)
        group = Group.objects.create(name=operation.operation_name+" operation join")
        assign_perm('join_operation', group, operation)
        self.user.groups.add(group)
        self.assertTrue(self.user.has_perm('join_operation', operation))
