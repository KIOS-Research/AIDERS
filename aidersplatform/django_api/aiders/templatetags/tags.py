from django import template

register = template.Library()

@register.filter
def removeUnderScore(string):
    string = string.replace('_', ' ')
    return string

@register.filter
def lookup(d, key):
    return d[key]