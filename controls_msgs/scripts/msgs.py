# import os

# def collect_text_until_separator(input_directory, output_directory):
#     # Ensure the output directory exists, create it if it doesn't
#     os.makedirs(output_directory, exist_ok=True)

#     # Loop through each file in the specified input directory
#     for filename in os.listdir(input_directory):
#         # Only process text files
#         if filename.endswith('.srv'):  # Change to the appropriate extension if necessary
#             file_path = os.path.join(input_directory, filename)
#             collected_lines = []

#             # Open the current file and read line by line
#             with open(file_path, 'r') as file:
#                 for line in file:
#                     if line.startswith('---'):
#                         break  # Stop collecting lines when '---' is encountered
#                     collected_lines.append(line)

#             # Prepare the new filename by adding 'Params' before the .msg extension
#             new_filename = f"{os.path.splitext(filename)[0]}SentParams.msg"
#             new_file_path = os.path.join(output_directory, new_filename)

#             # Write the collected lines to the new file in the output directory
#             with open(new_file_path, 'w') as new_file:
#                 new_file.writelines(collected_lines)

#             print(f"Processed '{filename}' and created '{new_filename}' in '{output_directory}'.")

# # Specify the directories
# input_directory = 'src/controls/controls_msgs/srv'
# output_directory = 'src/controls/controls_msgs/msg'

# collect_text_until_separator(input_directory, output_directory)

import os

def collect_text_after_separator(input_directory, output_directory):
    # Ensure the output directory exists, create it if it doesn't
    os.makedirs(output_directory, exist_ok=True)

    # Loop through each file in the specified input directory
    for filename in os.listdir(input_directory):
        # Only process .srv files
        if filename.endswith('.srv'):  # Change to the appropriate extension if necessary
            file_path = os.path.join(input_directory, filename)
            collected_lines = []
            after_separator = False  # Flag to indicate if we have passed the separator

            # Open the current file and read line by line
            with open(file_path, 'r') as file:
                for line in file:
                    if line.startswith('---'):
                        after_separator = True  # Set the flag to True after encountering '---'
                        continue  # Skip the '---' line itself
                    if after_separator:  # Only collect lines after the separator
                        collected_lines.append(line)

            # Prepare the new filename by adding 'SentParams' before the .msg extension
            new_filename = f"{os.path.splitext(filename)[0]}RecvParams.msg"
            new_file_path = os.path.join(output_directory, new_filename)

            # Write the collected lines to the new file in the output directory
            with open(new_file_path, 'w') as new_file:
                new_file.writelines(collected_lines)

            print(f"Processed '{filename}' and created '{new_filename}' in '{output_directory}'.")

# Specify the directories
input_directory = 'src/controls/controls_msgs/srv'
output_directory = 'src/controls/controls_msgs/msg'

collect_text_after_separator(input_directory, output_directory)
