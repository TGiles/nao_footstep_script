Instructions:
* Start up NAO by holding chest button
* Once lights turn on, release chest button
* NAO is ready when he says "OUNAK OUGH" (phonetically)
* Press chest button to determine IP. Should be 192.168.10.x
* Modify <pre>almotion_setFootSteps.py</pre> and update <pre>ROBOT_IP</pre>
    * Should be around line 6 and around line 211
* Go to location of <pre>almotion_setFootSteps.py</pre> and run the following command: <pre>python almotion_setFootSteps.py</pre>
* NAO should start walking based on the generated plan in the setFootSteps.py file