from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt

import json
from types import SimpleNamespace

@csrf_exempt
def index(request):
	try:
		# Parse JSON into an object with attributes corresponding to dict keys.
		x = json.loads(request.body, object_hook=lambda d: SimpleNamespace(**d)) # need to pass x to the app
		return JsonResponse({'status': 'request received!'})
	except:
		return JsonResponse({'status': 'something went wrong...'})
		
def download(request):
	return JsonResponse({'status': 'request received'})
