import os

def platform_version(request):
    return {"platform_version": os.environ.get("VERSION", "")}