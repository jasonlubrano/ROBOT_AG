	
#!/usr/bin/env python
import pandas as pd
plants =  pd.read_excel('plant.xlsx', sheet_name=1)
vals = plants.values
vals = vals[0]
plants = []
for i in range(1,5):
	plants.append(int(vals[i]))
print(plants)

