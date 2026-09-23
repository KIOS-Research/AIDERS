# from django import forms

from django.contrib.auth import get_user_model
from django.contrib.auth.forms import UserCreationForm
from django.contrib.gis import forms
from django.forms import DateTimeInput, ModelForm
from django.core.exceptions import ValidationError

from .models import Drone, FlyingReport, Operation, StaticCamera, User, WeatherConfig


class NewOperationFormForm(forms.Form, ModelForm):
    class Meta:
        model = Operation
        fields = ("operation_name", "location", "description")
        widgets = {
            "operation_name": forms.TextInput(attrs={"placeholder": "e.g Exercise_02Aug"}),
            "location": forms.TextInput(attrs={"placeholder": "e.g Aglantzia"}),
            "description": forms.Textarea(attrs={"placeholder": "The objectives and scenario of this operation", "rows": 4}),
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
            "start_date_time": DateTimeInput(attrs={"type": "datetime-local", "class": "form-control"}),
            "end_date_time": DateTimeInput(attrs={"type": "datetime-local", "class": "form-control"}),
        }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        for field_name, field in self.fields.items():
            field.widget.attrs['class'] = 'form-control'
        self.fields['latitude'].help_text = "The latitude of the operation's starting point."
        self.fields['longitude'].help_text = "The longitude of the operation's starting point."
        self.fields['altitude'].help_text = "The maximum flying altitude in meters."
        self.fields['radius'].help_text = "The maximum distance from the operation's starting point in meters."
        self.fields['buffer_altitude'].help_text = "Safety altitude buffer in meters."
        self.fields['buffer_radius'].help_text = "Safety distance buffer in meters."

        


class OperationReportForm(forms.Form):
    """Form for generating operation reports"""
    
    start_date = forms.DateTimeField(
        label="Start Date & Time",
        widget=forms.DateTimeInput(
            format='%Y-%m-%dT%H:%M',
            attrs={
                'type': 'datetime-local',
                'class': 'form-control'
            }
        ),
        required=False,
        help_text="Optional: Filter telemetry data from this date/time onwards. Defaults to the operation's start."
    )

    end_date = forms.DateTimeField(
        label="End Date & Time",
        widget=forms.DateTimeInput(
            format='%Y-%m-%dT%H:%M',
            attrs={
                'type': 'datetime-local',
                'class': 'form-control'
            }
        ),
        required=False,
        help_text="Optional: Filter telemetry data up to this date/time. Defaults to the operation's end."
    )
    
    include_flight_paths = forms.BooleanField(
        label="Include Flight Path Maps",
        required=False,
        initial=True,
        widget=forms.CheckboxInput(attrs={'class': 'form-check-input'}),
        help_text="Generate visual maps showing UAV flight paths"
    )
    
    include_statistics = forms.BooleanField(
        label="Include Flight Statistics",
        required=False,
        initial=True,
        widget=forms.CheckboxInput(attrs={'class': 'form-check-input'}),
        help_text="Include detailed flight statistics for each UAV"
    )
    
    enhanced_maps = forms.BooleanField(
        label="Enhanced Map Visualization",
        required=False,
        initial=True,
        widget=forms.CheckboxInput(attrs={'class': 'form-check-input'}),
        help_text="Use enhanced maps with topographic background and additional details"
    )

    def clean(self):
        cleaned_data = super().clean()
        start_date = cleaned_data.get('start_date')
        end_date = cleaned_data.get('end_date')

        if start_date and end_date and start_date >= end_date:
            raise forms.ValidationError(
                "Start date & time should be before end date & time."
            )

        return cleaned_data


class StaticCameraForm(ModelForm):
    class Meta:
        model = StaticCamera
        fields = ("name", "model", "live_stream_url", "latitude", "longitude", "operation", "is_connected_with_platform")
        widgets = {
            "name": forms.TextInput(attrs={"placeholder": "e.g Camera_01", "class": "form-control"}),
            "model": forms.TextInput(attrs={"placeholder": "e.g IP Camera Model XYZ", "class": "form-control"}),
            "live_stream_url": forms.URLInput(attrs={"placeholder": "rtmp://example.com/stream", "class": "form-control"}),
            "latitude": forms.NumberInput(attrs={"placeholder": "35.1667", "step": "any", "class": "form-control"}),
            "longitude": forms.NumberInput(attrs={"placeholder": "33.3667", "step": "any", "class": "form-control"}),
            "operation": forms.Select(attrs={"class": "form-control"}),
            "is_connected_with_platform": forms.CheckboxInput(attrs={"class": "form-check-input"}),
        }


class WeatherConfigForm(forms.ModelForm):
    class Meta:
        model = WeatherConfig
        fields = ['api_key', 'cities', 'update_interval', 'is_active']
        
        widgets = {
            'api_key': forms.TextInput(attrs={
                'class': 'form-control',
                'placeholder': 'Enter your WeatherAPI.com API key'
            }),
            'cities': forms.Textarea(attrs={
                'class': 'form-control',
                'rows': 4,
                'placeholder': 'Enter cities separated by commas (e.g., London, New York, Paris)'
            }),
            'update_interval': forms.NumberInput(attrs={
                'class': 'form-control',
                'min': 10,
                'max': 1440
            }),
            'is_active': forms.CheckboxInput(attrs={
                'class': 'form-check-input'
            })
        }
        
        help_texts = {
            'api_key': 'Get your free API key from weatherapi.com',
            'cities': 'Enter city names separated by commas',
            'update_interval': 'How often to fetch weather data (10-1440 minutes)',
            'is_active': 'Enable/disable automatic weather updates'
        }
