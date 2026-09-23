from flask import Flask, request, jsonify
import subprocess

app = Flask(__name__)

@app.route('/rebroadcast', methods=['POST'])
def rebroadcast():
    data = request.get_json()
    input_url = data.get('input_url')
    output_url = data.get('output_url')

    if not input_url or not output_url:
        return jsonify({'error': 'Both input_url and output_url are required'}), 400

    try:
        # Start FFmpeg process

        # simple copy
        # 'ffmpeg', '-i', input_url, '-c', 'copy', '-f', 'flv', output_url

        # timestamp correction
        # ffmpeg -i rtmp://.../live/teststream -avoid_negative_ts make_zero -fflags +genpts -c:v copy -c:a copy -f flv rtmp://.../live/stream
        # 'ffmpeg', '-i', input_url, '-avoid_negative_ts', 'make_zero', '-fflags', '+genpts', '-c:v', 'copy', '-c:a', 'copy', '-f', 'flv', output_url

        # buffer and timestamp filtering
        # ffmpeg -i rtmp://.../live/teststream -async 1 -vsync 1 -c:v copy -c:a aac -b:a 128k -ar 44100 -f flv rtmp://.../live/stream
        # 'ffmpeg', '-i', input_url, '-async', '1', '-vsync', '1', '-c:v', 'copy', '-c:a', 'aac', '-b:a', '128k', '-ar', '44100', '-f', 'flv', output_url

        # robust solution with re-encoding
        # ffmpeg -i rtmp://.../live/teststream -c:v libx264 -preset veryfast -c:a aac -b:a 128k -ar 44100 -r 30 -g 60 -keyint_min 60 -avoid_negative_ts make_zero -f flv rtmp://.../live/stream
        # 'ffmpeg', '-i', input_url, '-c:v', 'libx264', '-preset', 'veryfast', '-c:a', 'aac', '-b:a', '128k', '-ar', '44100', '-r', '30', '-g', '60', '-keyint_min', '60', '-avoid_negative_ts', 'make_zero', '-f', 'flv', output_url
        

        subprocess.Popen([
            'ffmpeg', '-i', input_url, '-c:v', 'h264_nvenc', '-preset', 'p4', '-bf', '0', '-b:v', '2M', '-an', '-f', 'flv', output_url
        ])
        return jsonify({'message': 'Rebroadcast started successfully'}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
