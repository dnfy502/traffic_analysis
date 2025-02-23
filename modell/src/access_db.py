#!/usr/bin/env python3
from pymongo import MongoClient

# Replace with your connection string
MONGO_URI = "mongodb+srv://insigniustwitch:VerWheLwUEEuJiDl@cluster0.rgt8z.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0"

# Connect to MongoDB Atlas
client = MongoClient(MONGO_URI)

# Select database
db = client["bingus"]  # Change to your database name

# Select collection
collection = db["bongus"]  # Change to your collection name

# Fetch documents with a filter
query = {"class": 2}  # Get all documents where age > 25
filtered_data = collection.find(query)
if __name__=="__main__":
    for doc in filtered_data: 
        print(doc)
