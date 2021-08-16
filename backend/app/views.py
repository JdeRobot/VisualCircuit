from json.decoder import JSONDecodeError

from django.http.response import HttpResponseBadRequest
from synthesis.synthesis import synthesize
from django.http import JsonResponse, FileResponse
from django.views.decorators.csrf import csrf_exempt

import json

@csrf_exempt
def build(request):
    try:
        data = json.loads(request.body)
        filename, file = synthesize(data)
        return FileResponse(file, as_attachment=True, filename=filename)
    except JSONDecodeError as e:
        return HttpResponseBadRequest(e.msg)



def download(request):
    return JsonResponse({'status': 'request received'})
