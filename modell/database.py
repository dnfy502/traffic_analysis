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

# Insert a single document
data = {"id": "1", "class":"car", "x":1.1,"y":2.1,"z":14.11}
inserted_id = collection.insert_one(data).inserted_id
print(f"Inserted document ID: {inserted_id}")