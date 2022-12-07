# from django import forms

from django.contrib.auth import get_user_model
from django.contrib.auth.forms import UserCreationForm
from django.contrib.gis import forms
from django.forms import DateTimeInput, ModelForm

from .models import Drone, FlyingReport, Operation, User


# Function to retrieve all the Username data
class UserModelChoiceField(forms.ModelChoiceField):
    def label_from_instance(self, obj):
        return "%s" % (obj.username)

# Function to retrieve all the Drone names data


class DroneModelChoiceField(forms.ModelChoiceField):
    def label_from_instance(self, obj):
        return "%s" % (obj.drone_name)


class OperationFormStep1(ModelForm):
    class Meta:
        model = Operation
        fields = ('operation_name', 'location', 'description')
        widgets = {
            'operation_name': forms.TextInput(attrs={'placeholder': 'e.g Exercise_02Aug'}),
            'location': forms.TextInput(attrs={'placeholder': 'e.g Aglantzia'}),
            'description': forms.Textarea(attrs={'placeholder': 'The objectives and scenario of this operation'}),
        }
        # widgets = {'location': GooglePointFieldWidget,}          #add in order to run the Google Map Point Field Widget


class OperationFormEditStep1(ModelForm):
    class Meta:
        model = Operation
        fields = ('location', 'description')
        # widgets = {'location': GooglePointFieldWidget,}           #add in order to run the Google Map Point Field Widget


class OperationFormStep2(forms.Form, ModelForm):
    users = UserModelChoiceField(User.objects.exclude(
        username="AnonymousUser"), required=False)
    drones = DroneModelChoiceField(Drone.objects.all(), required=False)

    class Meta:
        model = Operation
        fields = ('drones',)


class JoinOperationForm(ModelForm):
    operation_name = forms.CharField(max_length=100)


class NewUserForm(UserCreationForm):
    email = forms.EmailField(required=True)

    class Meta:
        model = get_user_model()
        fields = ("username",  "first_name", "last_name",
                  "email", "password1", "password2")

    def save(self, commit=True):
        user = super(NewUserForm, self).save(commit=False)
        user.email = self.cleaned_data['email']
        if commit:
            user.save()
        return user


class FlyingReportForm(ModelForm):
    class Meta:
        model = FlyingReport
        fields = ('latitude', 'longitude', 'altitude', 'radius',
                  'buffer_altitude', 'buffer_radius', 'start_date_time', 'end_date_time')
        widgets = {
            'start_date_time': DateTimeInput(attrs={'type': 'datetime-local'}),
            'end_date_time': DateTimeInput(attrs={'type': 'datetime-local'}),
        }
