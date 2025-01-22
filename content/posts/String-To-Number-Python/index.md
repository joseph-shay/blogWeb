---
title: "Convert Words to Numbers in Python"
date: 2025-01-22
draft: false
summary: "How to turn string into a Number with python and word2number. "
---

## Info

So the other day I came across this post on Instagram, and I decided to do it.
{{< figure
    src="post.jpg"
>}}
In this blog post, we’ll explore how to achieve this using Python and the powerful word2number library. By the end of this guide, you’ll be able to implement a simple function to handle such conversions in your projects.

---

### The Solution

The word2number library simplifies the process of converting number words into integers. Let’s dive straight into the implementation.

#### Step 1: Install the word2number Library

Before we begin, ensure you have the library installed. You can do this using pip:

```bash
pip install word2number
```

#### Step 2: Write the Conversion Function

Here’s a Python function that takes a string input and returns the corresponding number, formatted with commas for readability:

```python
from word2number import w2n

def string_to_number(input_string):
    try:
        number = w2n.word_to_num(input_string)
        formatted_number = f"{number:,}"
        return formatted_number
    except ValueError as e:
        return f"Error: {e}. Make sure the input string is a valid number."

while True:
    input_string = input("Enter string (type 'close' to exit): ")
    if input_string.lower() == "close":
        print("Exiting program...")
        break
    output = string_to_number(input_string)
    print(f"Input: {input_string}")
    print(f"Output: {output}")
```

---

#### Step 3: Test the Function

Let’s see the function in action. For the input "Three hundred million", the output will be:

```plaintext
Input: Three hundred million
Output: 300,000,000
```

---

## How It Works

Let’s break down the code step by step:

### Importing the Required Library
```python
from word2number import w2n
```
This line imports the `w2n` module from the `word2number` library, which provides the functionality to convert words into numbers.

### Defining the Function
```python
def string_to_number(input_string):
```
This defines a function named `string_to_number` that takes a single argument `input_string`, which is expected to be a string containing a number written in words.

### Try Block: Conversion and Formatting
```python
    try:
        number = w2n.word_to_num(input_string)
        formatted_number = f"{number:,}"
        return formatted_number
```
- **`w2n.word_to_num(input_string)`**: This converts the input string (e.g., "three hundred") into an integer (e.g., `300`).
- **`f"{number:,}"`**: Formats the resulting number with commas for readability (e.g., `300000` becomes `300,000`).
- **`return formatted_number`**: Returns the formatted number as a string.

### Handling Invalid Input
```python
    except ValueError as e:
        return f"Error: {e}. Make sure the input string is a valid number."
```
- If the input string cannot be interpreted as a number, the `w2n.word_to_num()` function raises a `ValueError`. 
- The `except` block catches this error and returns a friendly error message indicating the issue.

### Infinite Loop for User Input
```python
while True:
    input_string = input("Enter string (type 'close' to exit): ")
```
- This creates an infinite loop to repeatedly prompt the user for input until they decide to exit.

### Exit Condition
```python
    if input_string.lower() == "close":
        print("Exiting program...")
        break
```
- Converts the input to lowercase using `input_string.lower()` and checks if it equals "close". 
- If true, the program prints an exit message and breaks the loop.

### Processing the Input
```python
    output = string_to_number(input_string)
    print(f"Input: {input_string}")
    print(f"Output: {output}")
```
- Calls the `string_to_number()` function with the user’s input.
- Prints both the original input and the function’s output (formatted number or error message).

---

## Final Thoughts

Converting words to numbers doesn’t have to be complicated. With Python’s word2number library, you can implement a robust solution in just a few lines of code. This function is perfect for applications in data processing, user input handling, or any scenario requiring natural language number conversion.

Feel free to adapt this function to suit your specific needs, and don’t forget to test it with various inputs to ensure reliability. Happy coding!
