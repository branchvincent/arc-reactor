import os
import json
import random

def generate_orders(n,box_fit_item,box_volume,item_dictionary,box_name):
		if n==2:
			box=random.randint(0,1)
		elif n==3:
			box=random.randint(2,3)
		else:
			box=4
		fitting_item=box_fit_item[box]
		items=random.sample(xrange(0,len(fitting_item)-1),n)
		box_v=box_volume[box]
		item_v=0
		for item in items:
			item_v=item_v+item_dictionary[fitting_item[item]]
		while item_v>box_v or item_v/box_v<0.5:
			items=random.sample(xrange(0,len(fitting_item)-1),n)
			box_v=box_volume[box]
			item_v=0
			for item in items:
				item_v=item_v+item_dictionary[fitting_item[item]]
		print "size_id:",box_name[box]
		print "item volume/box volume:",item_v/box_v*100
		for item in items:
			print fitting_item[item]






if __name__ == "__main__":
	with open("pick_task_generator.json") as json_file:
			data=json.load(json_file)
	box_fit_item=data['box_fit_item']
	box_volume=data['box_volume']
	item_dictionary=data['item_dictionary']
	box_name=data['box_name']
	#generate test cases
	#2 items order
	generate_orders(2,box_fit_item,box_volume,item_dictionary,box_name)
	generate_orders(3,box_fit_item,box_volume,item_dictionary,box_name)
	generate_orders(5,box_fit_item,box_volume,item_dictionary,box_name)

	