from json.decoder import JSONDecodeError

from django.http.response import HttpResponseBadRequest
from synthesis.synthesis import synthesize
from django.http import JsonResponse, FileResponse
from django.views.decorators.csrf import csrf_exempt

import json
import os
from django.conf import settings

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


@csrf_exempt
def install_block(request):
    try:
        data = json.loads(request.body)
        block_name = data.get('package', {}).get('name') or data.get('name', 'Untitled')
        
        custom_blocks_dir = os.path.join(settings.BASE_DIR, 'custom_blocks')
        if not os.path.exists(custom_blocks_dir):
            os.makedirs(custom_blocks_dir)
            
        safe_filename = "".join([c for c in block_name if c.isalpha() or c.isdigit() or c in (' ')]).rstrip()
        filepath = os.path.join(custom_blocks_dir, f"{safe_filename}.vc3")
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=4)
            
        return JsonResponse({'status': 'success', 'saved_as': os.path.basename(filepath)})
    except Exception as e:
        return HttpResponseBadRequest(str(e))

def installed_blocks(request):
    try:
        custom_blocks_dir = os.path.join(settings.BASE_DIR, 'custom_blocks')
        if not os.path.exists(custom_blocks_dir):
            return JsonResponse({'blocks': []})
            
        blocks = []
        for filename in os.listdir(custom_blocks_dir):
            if filename.endswith('.vc3'):
                filepath = os.path.join(custom_blocks_dir, filename)
                try:
                    with open(filepath, 'r') as f:
                        block_data = json.load(f)
                        blocks.append(block_data)
                except Exception as e:
                    print(f"Failed to load block {filename}: {e}")
                    
        return JsonResponse({'blocks': blocks})
    except Exception as e:
        return HttpResponseBadRequest(str(e))
