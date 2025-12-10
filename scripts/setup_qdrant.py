import sys
from pathlib import Path

# Add project root to sys.path
project_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_root))

from qdrant_client import QdrantClient, models
from api.config import settings

def setup_qdrant():
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
        timeout=60,
    )

    collection_name = "book_content"

    # Check if the collection already exists
    try:
        collection_info = client.get_collection(collection_name=collection_name)
        print(f"Collection '{collection_name}' already exists.")
        return
    except Exception:
        # Collection does not exist, so create it
        print(f"Collection '{collection_name}' does not exist. Creating it now.")

    client.recreate_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
    )
    print(f"Collection '{collection_name}' created successfully.")

if __name__ == "__main__":
    setup_qdrant()
