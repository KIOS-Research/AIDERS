from django.contrib.auth import get_user_model
from django.test import Client
from rest_framework.test import APIRequestFactory, APITestCase


class TestSetUp(APITestCase):
    def setUp(self):
        self.username = 'testuser'
        self.password = 'asd123ASD!@#'
        c = Client()
        self.operation_name = 'newOperation0'
        self.factory = APIRequestFactory()
        self.user = get_user_model().objects.create(username=self.username)
        self.user.set_password(self.password)
        self.user.save()
        self.drone_name = "kios_mavicTest"
        self.model = "mavicTest"
        self.logged_in = c.login(username=self.username,password=self.password)
        self.content_type = "application/json"
        self.assertTrue(self.logged_in)
        return super().setUp()


    def tearDown(self):
        return super().tearDown()
