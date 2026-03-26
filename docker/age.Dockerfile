FROM postgres:15

# Install build dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    postgresql-server-dev-15 \
    flex \
    bison \
    && rm -rf /var/lib/apt/lists/*

# Build Apache AGE
RUN git clone --branch release/PG15/1.6.0 --single-branch \
        https://github.com/apache/age.git /age \
    && cd /age \
    && make PG_CONFIG=/usr/lib/postgresql/15/bin/pg_config \
    && make install PG_CONFIG=/usr/lib/postgresql/15/bin/pg_config

# Ensure AGE loads
RUN echo "shared_preload_libraries = 'age'" >> /usr/share/postgresql/postgresql.conf.sample

# Init script is mounted via docker-compose volume
