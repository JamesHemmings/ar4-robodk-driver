# Python
from flask import Flask, request, render_template, redirect, url_for, session

app = Flask(__name__)
app.secret_key = 'your secret key'  # replace with your secret key

@app.route('/', methods=['GET'])
def home():
    return render_template('form.html', previous_message=session.get('message') or 'Input here')

@app.route('/send_message', methods=['GET', 'POST'])
def send_message():
    if request.method == 'POST':
        message = request.form.get('message')
        session['message'] = message  # store the message in session
        print(message)  # print the message to the terminal
        return '''
            <html>
                <body>
                    <p>Message received: {}</p>
                    <script>
                        setTimeout(function(){{
                            window.location.href = "/";
                        }}, 3000);
                    </script>
                </body>
            </html>
        '''.format(message)
    else:
        return render_template('form.html', previous_message=session.get('message'))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)