# from django import forms

from django.contrib.auth import get_user_model
from django.contrib.auth.forms import UserCreationForm
from django.contrib.gis import forms
from django.forms import DateTimeInput, ModelForm

from .models import Drone, FlyingReport, Operation, User


class NewOperationFormForm(forms.Form, ModelForm):
    class Meta:
        model = Operation
        fields = ("operation_name", "location", "description", "disaster_epicenter_latitude", "disaster_epicenter_longtitude", "dense_area_of_buildings", "max_extreme_temperature", "risk_of_explosion_and_fire")
        widgets = {
            "operation_name": forms.TextInput(attrs={"placeholder": "e.g Exercise_02Aug"}),
            "location": forms.TextInput(attrs={"placeholder": "e.g Aglantzia"}),
            "description": forms.Textarea(attrs={"placeholder": "The objectives and scenario of this operation"}),
        }


class JoinOperationForm(ModelForm):
    operation_name = forms.CharField(max_length=100)


class NewUserForm(UserCreationForm):
    email = forms.EmailField(required=True)

    class Meta:
        model = get_user_model()
        fields = ("username", "first_name", "last_name", "email", "password1", "password2")

    def save(self, commit=True):
        user = super(NewUserForm, self).save(commit=False)
        user.email = self.cleaned_data["email"]
        if commit:
            user.save()
        return user


class FlyingReportForm(ModelForm):
    class Meta:
        model = FlyingReport
        fields = ("latitude", "longitude", "altitude", "radius", "buffer_altitude", "buffer_radius", "start_date_time", "end_date_time")
        widgets = {
            "start_date_time": DateTimeInput(attrs={"type": "datetime-local"}),
            "end_date_time": DateTimeInput(attrs={"type": "datetime-local"}),
        }
