'''
need to add messages for sending can msgs, 
only service needed is can message service type. 
motor node accepts and parses and publishes out. 
control nodes publish to msgs, motor node sends service request. 
OR service request to motor node, motor node does service request


DO THIS:
    - we get data where we want it by default, (no additional control sub to data)
    - also get data everywhere else (yay)

control -> srv -> motor -> srv -> can
                   |
                   |
                   |
                   V
              data topics 
'''


