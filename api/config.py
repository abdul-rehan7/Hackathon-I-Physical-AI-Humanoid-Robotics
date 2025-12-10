from pydantic_settings import BaseSettings, SettingsConfigDict
from pathlib import Path

class Settings(BaseSettings):
    GEMINI_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str

    model_config = SettingsConfigDict(
        env_file=str(Path(__file__).parent.parent / ".env"),
        env_file_encoding='utf-8'
    )

settings = Settings()
