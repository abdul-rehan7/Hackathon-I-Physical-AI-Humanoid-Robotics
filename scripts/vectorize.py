import sys
import os
import glob
from pathlib import Path
from qdrant_client import QdrantClient, models
from fastembed import TextEmbedding
import uuid

# Add project root to sys.path
project_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_root))

from api.config import settings

def vectorize_docs():
    """
    Reads all markdown files from the docs/ directory, generates embeddings,
    and upserts them into the Qdrant collection.
    """
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
        timeout=60,
    )
    embedding_model = TextEmbedding()
    collection_name = "book_content"

    # Get all markdown files
    md_files = glob.glob(os.path.join(project_root, "docs/**/*.md"), recursive=True)
    mdx_files = glob.glob(os.path.join(project_root, "docs/**/*.mdx"), recursive=True)
    all_files = md_files + mdx_files

    texts = []
    metadatas = []
    for file_path in all_files:
        with open(file_path, "r", encoding="utf-8", errors='ignore') as f:
            content = f.read()
            # Simple chunking by paragraphs
            chunks = content.split("\n\n")
            for chunk in chunks:
                if chunk.strip():
                    texts.append(chunk)
                    metadatas.append({"source": os.path.relpath(file_path, project_root)})

    print(f"Found {len(texts)} document chunks to vectorize.")

    # Generate embeddings
    embeddings = list(embedding_model.embed(texts, batch_size=32))

    # Create and upsert points in batches
    batch_size = 100
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i+batch_size]
        batch_metadatas = metadatas[i:i+batch_size]
        batch_embeddings = embeddings[i:i+batch_size]

        points = []
        for j, (text, meta) in enumerate(zip(batch_texts, batch_metadatas)):
            points.append(
                models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=batch_embeddings[j].tolist(),
                    payload={
                        "text": text,
                        "source": meta["source"],
                    }
                )
            )

        client.upsert(
            collection_name=collection_name,
            points=points,
            wait=True,
        )
        print(f"Upserted batch {i//batch_size + 1}/{(len(texts) + batch_size - 1)//batch_size}")


    print("Vectorization complete.")

if __name__ == "__main__":
    vectorize_docs()
