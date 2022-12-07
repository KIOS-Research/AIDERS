from aiders.factories import OperationFactory
from aiders.forms import (NewUserForm, OperationFormEditStep1,
                          OperationFormStep1)
from django.contrib.gis.geos import Point

from .test_setup import TestSetUp


class TestForms(TestSetUp):
    def test_create_operation_form_step1_that_already_exists(self):
        random_lat = 32.12
        random_lon = 21.42
        op = OperationFactory.create(operation_name=self.operation_name, operator=self.user)

        form = OperationFormStep1(
            data = {
                'description':'this is a test description to test that valid data are entered on this form',
                'location':'location',
                'operation_name':self.operation_name
            }
        )

        self.assertFalse(form.is_valid())
        self.assertIn('operation_name', form.errors.keys())
        self.assertIn('already exists', form.errors['operation_name'][0])

    def test_create_operation_form_step1_valid_data(self):
        random_lat = 32.12
        random_lon = 21.42

        form = OperationFormStep1(
            data = {
                'description':'this is a test description to test that valid data are entered on this form',
                # 'location':{"point": "SRID=4326;POINT ({} {})".format(random_lat, random_lon)},
                'location':'location',
                'operation_name':self.operation_name
            }
        )
        self.assertTrue(form.is_valid())

    def test_create_operation_form_step1_invalid_data(self):
        number_of_fields_required= 2
        form = OperationFormStep1(data={})
        self.assertFalse(form.is_valid())
        self.assertEquals(len(form.errors),number_of_fields_required)

    def test_edit_operation_form_step1_valid_data(self):
        random_lat = 32.12
        random_lon = 21.42

        form = OperationFormEditStep1(
            data = {
                'description':'this is a test description to test that valid data are entered on this form',
                # 'location':{"point": "SRID=4326;POINT ({} {})".format(random_lat, random_lon)},
                'location':'location',
            }
        )
        self.assertTrue(form.is_valid())

    def test_edit_operation_form_step1_invalid_data(self):
        number_of_fields_required= 1
        form = OperationFormEditStep1(data={})
        self.assertFalse(form.is_valid())
        self.assertEquals(len(form.errors),number_of_fields_required)

    def test_create_new_user_form_valid_data(self):
        form = NewUserForm(
            data = {
                'username':'testUserName',
                'email':'testEmail@gmail.com',
                'password1':'randomPw123',
                'password2':'randomPw123'
            }
        )
        self.assertTrue(form.is_valid())

    def test_create_new_user_form_invalid_data(self):
        correct_data =  {
                'username':'testUserName',
                'email':'testEmail@gmail.com',
                'password1':'randomPw123',
                'password2':'randomPw123'
            }

        self.assertTrue((NewUserForm(data=correct_data)).is_valid())

        data_wrong_confirmed_pw = {
                'username':'testUserName',
                'email':'testEmail@gmail.com',
                'password1':'randomPw123',
                'password2':'dd'
            }
        self.assertFalse((NewUserForm(data=data_wrong_confirmed_pw)).is_valid())

        data_not_strong_pw = {
            'username': 'testUserName',
            'email': 'testEmail@gmail.com',
            'password1': '123',
            'password2': '123'
        }

        self.assertFalse((NewUserForm(data=data_not_strong_pw)).is_valid())


        data_not_correct_email_format = {
            'username': 'testUserName',
            'email': 'testEmail',
            'password1': '123',
            'password2': '123'
        }

        self.assertFalse((NewUserForm(data=data_not_correct_email_format)).is_valid())

    # def test_join_operation_exists(self):
    #     OperationFactory.create(operation_name=self.operation_name,
    #                                  operator=self.user)
    #     form = JoinOperationForm(
    #         data = {'operation_name':self.operation_name}
    #     )
    #     self.assertTrue(form.is_valid())
