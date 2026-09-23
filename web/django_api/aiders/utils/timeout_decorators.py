"""
Timeout decorators and utilities for long-running operations
"""

import time
import functools
import threading
from django.http import JsonResponse
from django.conf import settings


class TimeoutError(Exception):
    """Custom timeout exception"""
    pass


def with_timeout(timeout_seconds=None):
    """
    Decorator to add basic timeout tracking to Django views
    Note: This doesn't actually interrupt the operation, but provides
    timeout configuration that can be used by nginx/gunicorn
    
    Args:
        timeout_seconds: Timeout in seconds. If None, uses settings.REQUEST_TIMEOUT
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(request, *args, **kwargs):
            # Get timeout from parameter or settings
            timeout = timeout_seconds or getattr(settings, 'REQUEST_TIMEOUT', 300)
            
            # Record start time for logging
            start_time = time.time()
            
            try:
                result = func(request, *args, **kwargs)
                
                # Log execution time
                elapsed_time = time.time() - start_time
                print(f"Operation completed in {elapsed_time:.2f} seconds (timeout set to {timeout}s)")
                
                return result
            except Exception as e:
                elapsed_time = time.time() - start_time
                print(f"Operation failed after {elapsed_time:.2f} seconds: {str(e)}")
                raise e
        
        return wrapper
    return decorator


def with_progress_tracking():
    """
    Decorator to add progress tracking for long-running operations
    This is a placeholder for future implementation
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(request, *args, **kwargs):
            # Future: Add progress tracking logic here
            return func(request, *args, **kwargs)
        return wrapper
    return decorator
