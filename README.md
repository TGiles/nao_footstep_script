Instructions:
1. Start up NAO by holding chest button
2. Once lights turn on, release chest button
3. NAO is ready when he says "OUNAK OUGH" (phonetically)
4. Press chest button to determine IP.
5. cd to nao_footstep_script location
6. Run the following command (--ip is determined by step 4) :
<pre>
python proto_walking_controller.py --ip '192.168.10.110'
</pre>
7. Nao should now walk the <pre>desired_distance</pre> value set in the walking controller (currently 0.5191m)
8. After finishing the plan (or falling over), the plotter should generate a plot of the executed footsteps and body pose.
* Additionally, to run the plotter manually:
<pre>
python plot_footsteps.py --test_dir='whatever your test dir is' --exp_dir='if your experiment dir is different than default'
</pre>
* Easiest way to get test_dir is to
<pre>
cd experiment_data/ [tab to timestamp test directory]
delete the experiment_data/
delete the / on the end of timestamp test dir
</pre>
