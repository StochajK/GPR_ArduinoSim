"""
Kimberly Stochaj
3 April 2022
GPR.py
"""
# import necessary libraries
import csv

# initialize necessary constants
FILE = "GPR_MaterialData.csv"

def main():
    material = []
    conductivity = []
    relative_permiability = []
    attenuation = []
    
    with open(FILE, "r") as infile:
        csvfile = csv.reader(infile, delimiter = ",")
        next(csvfile)
        for row in csvfile:
            material.append(row[0])
            conductivity.append((float(row[1]) + float(row[2])) / 2)
            relative_permiability.append((float(row[3]) + float(row[4])) / 2)
            attenuation.append((float(row[5]) + float(row[5])) / 2)
    
    print(len(material))
    print(len(conductivity))
    print(len(relative_permiability))
    print(len(attenuation))
    
    u_conductivity = float(input("Conductivity:"))
    u_relative_permiability = float(input("Relative permiability:"))
    u_attenuation = float(input("Attenuation:"))
    
    most_similar = 10000
    u_material = ""
    
    for i in range(len(material)):
        euclidian = (((u_conductivity - conductivity[i]) ** 2) + 
                       ((u_relative_permiability - relative_permiability[i]) ** 2) + 
                       ((u_attenuation - attenuation[i]) ** 2) ** (1/2))
        if euclidian < most_similar:
            most_similar = euclidian
            u_material = material[i]
            
    print("The material is most likely", u_material, "with a similarity value of", most_similar)
            
        
    
    
main()
