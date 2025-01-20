import ollama
import re
# Get the user's input object name
object_name = input("Please enter the object name: ")

# Embed the hierarchical information and value information into the message content
message_content = f"""
Based on the following dictionaries, determine which level the object '[object_name]' is most similar to, and output only the corresponding value of that level as a single number. If the object is not listed, use your best judgment to determine the closest match and output the value for that level. Do not include any code or text, only a number.

Dictionary of hierarchical structures and corresponding object arrays:

LayerA: ["hot dog", "pizza", "donut", "cake"]  #  Soft Texture Items

LayerB: [ "banana", "orange"]  # Moderately Soft Texture Items

LayerC: ["broccoli", "carrot","apple"]  # Hard Texture Items

LayerD: ["bottle", "fork", "knife", "spoon"]  # Hard items

Dictionary of hierarchical structures and corresponding values:

    'LayerA': 1000,
    'LayerB': 2000,
    'LayerC': 3000,
    'LayerD': 4000,
    'LayerE': 5000


Please use the above information to determine which level the object '{object_name}' belongs to and output only the corresponding value of that level as a single number. Do not include any code or text, only a number.
"""

message = {
    "role": "assistant",
    "content": message_content
}

# Use ollama to interact with the large model
response = ollama.chat(model="llama3.2", stream=False, messages=[message], options={"temperature": 0.5})

# 从响应中提取content
content = response['message']['content']

print(content)
# 使用正则表达式匹配数字
numbers = re.findall(r'\d+', content)
if numbers:
    # 输出找到的第一个数字
    print(numbers[0])
else:
    print("无法从响应中提取数字")