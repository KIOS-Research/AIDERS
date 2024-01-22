from aiders.factories import OperationFactory
from aiders.forms import (NewUserForm,                           )

from .test_setup import TestSetUp


class TestForms(TestSetUp):

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
