-- Initialize Apache AGE extension for graph queries
CREATE EXTENSION IF NOT EXISTS age;
LOAD 'age';
SET search_path = ag_catalog, "$user", public;

-- Create the default graph for assignment-3
SELECT create_graph('semantic_map');
