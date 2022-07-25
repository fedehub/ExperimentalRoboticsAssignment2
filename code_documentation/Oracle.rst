
Code docs -- *simulation.cpp*
==============================


HOW TO receive a hint
----------------------

the node publishes the hint (message *erl2/ErlOracle*) through the topic */oracle_hint*. Here's the message prototype:

.. code-block::
	
	int32 ID
	string key
	string value

the hint is sent each time the *cluedo_link* is near enough to a marker. 

.. note::
	the oracle could send wrong informations, to simulate a disturbance in the hint recognition by the sensing part. 
	
	* the field "key" could be empty
	* the field "value" vould be empty
	* the field "key" could contain "-1", and also "value" could 


HOW TO check if the hint is valid
----------------------------------

to check if a ID is the solution of the mystery, call the service */oracle_solution* of type *erl2/Oracle*. Here's the prototype of the service:

.. code-block::
	
	---
	int32 ID


