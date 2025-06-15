import openai
import re
import string
import numpy as np
import difflib

class myTextCleanerWithOpenAI():
    def __init__(self, txtfile='openai_api_key.txt'):
        try:
            # Load the OpenAI API key from the file
            with open(txtfile) as f:
                api_key = f.readline().strip()
        except Exception as err:
            print(err)
            return
        
        openai.api_key = api_key


    def text_correcter(self, text):
        # Define the prompt for the AI, instructing it to correct the provided text.
        system_role = "Correct the following text and return the corrected version. Do not provide explanations, just return the corrected text. The corrected text should be comprised of most commonly used words. If the text is already correct, return the original text."
        messages = [
            {"role": "system", "content": system_role},
            {"role": "user", "content": text}
        ]
        
        # Send the correction request to the API.
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=messages
        )

        chat_response = completion.choices[0].message.content
        return chat_response

    def remove_commas_and_dots(self, text):
        return text.replace(',', '').replace('.', '')

    def remove_space_before_punctuation(self, text):
        pattern = r'\s+([.?!,:;])'
        return re.sub(pattern, r'\1', text)

    def count_words(self, s):
        return sum([i.strip(string.punctuation).isalpha() for i in s.split()])

    def run(self, text_wErrors):
        text_wErrors = self.remove_space_before_punctuation(text_wErrors)
        text_corrected = self.text_correcter(text_wErrors)
        
        # 교정 부호 제거
        text_wErrors_no_commas_dots = self.remove_commas_and_dots(text_wErrors)
        text_corrected_no_commas_dots = self.remove_commas_and_dots(text_corrected)
        
        # 변환 후 변화가 발생하였는지 확인
        isWordCountSame = np.abs(self.count_words(text_wErrors_no_commas_dots) - self.count_words(text_corrected_no_commas_dots)) < 2
        isError = isWordCountSame and (text_corrected_no_commas_dots.lower() != text_wErrors_no_commas_dots.lower())
        text_corrected = self.remove_space_before_punctuation(text_corrected)

        # 토큰화
        tokens1 = re.findall(r'\w+(?:-\w+)*\.?|\.', text_wErrors)
        tokens2 = re.findall(r'\w+(?:-\w+)*\.?|\.', text_corrected)

        #비교용
        d = difflib.Differ()
        diff = list(d.compare(tokens1, tokens2))

        text_with_added_markers = []
        for token in diff:
            if token.startswith('+ '):
                text_with_added_markers.append(token[2:])
            elif token.startswith('  '):
                text_with_added_markers.append(token[2:])
                
        reconstructed_corrected = ' '.join(text_with_added_markers)
        return reconstructed_corrected
