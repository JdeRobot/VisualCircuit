from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt
from django.http import JsonResponse

from math import floor, ceil

import json

from rest_framework.decorators import api_view

@api_view(('POST',))
@csrf_exempt
# Create your views here.
def calc(request):
    input = json.loads(request.body)
    sale_price = input.get('SP')
    cost = input.get('COST')
    if (sale_price is None) or (cost is None):
        return JsonResponse("Both sale price and cost are required.")
    else:
        profit = floor(sale_price - cost)
        if sale_price == 0: 
            sale_price = 1
        profit_margin = floor(profit / sale_price * 100)
        tax = ceil(profit * 0.2)
        data = {'profit': profit, 'profit_margin': profit_margin, 'tax': tax}
        return JsonResponse(data)