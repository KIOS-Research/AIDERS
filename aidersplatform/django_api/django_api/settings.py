"""
Django settings for django_api project.

Generated by 'django-admin startproject' using Django 3.2.9.

For more information on this file, see
https://docs.djangoproject.com/en/3.2/topics/settings/

For the full list of settings and their values, see
https://docs.djangoproject.com/en/3.2/ref/settings/
"""

from pathlib import Path
import os
from dotenv import load_dotenv

# Build paths inside the project like this: BASE_DIR / 'subdir'.
BASE_DIR = Path(__file__).resolve().parent.parent

load_dotenv(os.path.join(BASE_DIR, '.env.dev'))

# Quick-start development settings - unsuitable for production
# See https://docs.djangoproject.com/en/3.2/howto/deployment/checklist/

# SECURITY WARNING: keep the secret key used in production secret!
SECRET_KEY = os.environ.get('SECRET_KEY')

# SECURITY WARNING: don't run with debug turned on in production!
DEBUG = int(os.environ.get("DEBUG", default=0))

# APPEND_SLASH = False
ALLOWED_HOSTS = os.environ.get("DJANGO_ALLOWED_HOSTS").split(" ")

# Application definition

INSTALLED_APPS = [
    'django.contrib.admin',
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.messages',
    'django.contrib.staticfiles',
    'rest_framework',
    'floppyforms',
    'formtools',
    'guardian',
    'aiders',
    'django.contrib.gis',
    'crispy_forms',
    'mapwidgets',
    'drf_yasg',  # For API documentation generation
    'logic',
    'django_extensions',
    'channels'


]
CRISPY_TEMPLATE_PACK = 'bootstrap4'

MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    'django.contrib.messages.middleware.MessageMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware'
]
AUTHENTICATION_BACKENDS = (
    'django.contrib.auth.backends.ModelBackend',
    'guardian.backends.ObjectPermissionBackend'
)

AUTH_USER_MODEL = 'aiders.User'
ROOT_URLCONF = 'django_api.urls'

TEMPLATES = [
    {
        'BACKEND': 'django.template.backends.django.DjangoTemplates',
        'DIRS': [],
        'APP_DIRS': True,
        'OPTIONS': {
            'context_processors': [
                'django.template.context_processors.debug',
                'django.template.context_processors.request',
                'django.contrib.auth.context_processors.auth',
                'django.contrib.messages.context_processors.messages',
            ],
        },
    },
]

WSGI_APPLICATION = 'django_api.wsgi.application'
ASGI_APPLICATION = 'django_api.asgi.application'

CHANNEL_LAYERS = {
    'default': {
        'BACKEND': 'channels.layers.InMemoryChannelLayer',
    }
}

# REST_FRAMEWORK = {
#     'DEFAULT_PAGINATION_CLASS': 'rest_framework.pagination.PageNumberPagination',
#     'PAGE_SIZE': 10
# }

# Database
# https://docs.djangoproject.com/en/3.2/ref/settings/#databases

DATABASES = {
    # 'default': {
    #     'ENGINE': 'django.contrib.gis.db.backends.mysql',
    #     'NAME': 'testdb',
    #     'USER': 'repl',
    #     'PASSWORD': 'slavepass',
    #     'HOST': 'george',
    #     'PORT': '3306',
    # },
    'default': {

        "ENGINE": os.environ.get("SQL_ENGINE", "django.db.backends.sqlite3"),
        "NAME": os.environ.get("SQL_DATABASE", os.path.join(BASE_DIR, "db.sqlite3")),
        "USER": os.environ.get("SQL_USER", "user"),
        "PASSWORD": os.environ.get("SQL_PASSWORD", "password"),
        "HOST": os.environ.get("SQL_HOST", "localhost"),
        "PORT": os.environ.get("SQL_PORT", "5432")
    }

}


# Password validation
# https://docs.djangoproject.com/en/3.2/ref/settings/#auth-password-validators

AUTH_PASSWORD_VALIDATORS = [
    {
        'NAME': 'django.contrib.auth.password_validation.UserAttributeSimilarityValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.MinimumLengthValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.CommonPasswordValidator',
    },
    {
        'NAME': 'django.contrib.auth.password_validation.NumericPasswordValidator',
    },
]


# Internationalization
# https://docs.djangoproject.com/en/3.2/topics/i18n/

LANGUAGE_CODE = 'en-us'

TIME_ZONE = 'Asia/Famagusta'

USE_I18N = True

USE_L10N = True

USE_TZ = True

LOGGING = {
    'version': 1,                       # the dictConfig format version
    'disable_existing_loggers': False,  # retain the default loggers

    'handlers': {
        'general_file': {
            'class': 'logging.FileHandler',
            'filename': 'general.log',
            'formatter': 'verbose',
        },
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'simple',
        },
    },

    'loggers': {
        '': {
            'level': 'INFO',
            # To stop logging showing in console remove from the handlers
            'handlers': ['general_file', ],
        },
    },

    'formatters': {
        'verbose': {
            'format': '{asctime} {levelname} {name} {module} {message}',
            # 'format': '{levelname} {name} {asctime} {module} {process:d} {thread:d} {message}',
            'style': '{',
        },
        'simple': {
            'format': '{asctime} {levelname} {message}',
            'style': '{',
        },
    },
}
# Static files (CSS, JavaScript, Images)
# https://docs.djangoproject.com/en/3.2/howto/static-files/

STATIC_URL = '/static/'
STATIC_ROOT = os.path.join(BASE_DIR, 'aiders', 'static')
MEDIA_URL = '/media/'
# todo: Add checks to check if every following path exists
ALGORITHM_OUTPUTS_URL = os.path.join(MEDIA_URL, 'algorithm_outputs')
OBJECT_3D_OUTPUTS_URL = os.path.join(ALGORITHM_OUTPUTS_URL, '3d_objects')

AIDERS_DIR = os.path.join(BASE_DIR, 'aiders')
MEDIA_ROOT = os.path.join(AIDERS_DIR, 'media')

ALGORITHM_OUTPUTS_DIR = os.path.join(MEDIA_ROOT, 'algorithm_outputs')
OBJECT_3D_OUTPUTS_DIR = os.path.join(ALGORITHM_OUTPUTS_DIR, '3d_objects')


# ALGORITHM_OUTPUTS_DIR = os.path.join(MEDIA_ROOT, 'algorithm_outputs')
# Default primary key field type
# https://docs.djangoproject.com/en/3.2/ref/settings/#default-auto-field

DEFAULT_AUTO_FIELD = 'django.db.models.BigAutoField'

# Once user logs in, redirect them to the main html page of the aiders platform, which is the map
LOGIN_REDIRECT_URL = 'home'
LOGIN_URL = 'login'
DATA_UPLOAD_MAX_MEMORY_SIZE = 5242880
'''
Might need this in the future?
It is a google maps widget that lets the user select a location
A good use case is when an operator creates a new operation and is required to select a location
'''

# Google Map Point Field Widget

# MAP_WIDGETS = {
#     "GooglePointFieldWidget": (
#         ("zoom", 15),
#         ("mapCenterLocationName", "london"),
#         ("GooglePlaceAutocompleteOptions", {'componentRestrictions': {'country': 'uk'}}),
#         ("markerFitZoom", 12),
#     ),
#     "GOOGLE_MAP_API_KEY": "AIzaSyDaVSpIs0dsD28kR3H_2KoGc0zpEaeqsVo"
# }

# GOOGLE_MAPS_V3_APIKEY = "AIzaSyDaVSpIs0dsD28kR3H_2KoGc0zpEaeqsVo"
