-- Initialize pgvector extension for embedding storage
CREATE EXTENSION IF NOT EXISTS vector;

-- Detection embeddings table (referenced by assignment-3)
CREATE TABLE IF NOT EXISTS detection_embeddings (
  det_pk bigserial PRIMARY KEY,
  run_id text,
  det_id text UNIQUE,
  keyframe_id integer,
  class_name text NOT NULL,
  confidence real NOT NULL,
  bbox real[] NOT NULL,
  map_x real,
  map_y real,
  map_yaw real,
  embedding_model text,
  embedding vector(512),
  ingested_at timestamptz NOT NULL DEFAULT now()
);
