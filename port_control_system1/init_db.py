import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT
import json
import os
from pathlib import Path

# JSON 파일 경로
_APP_DIR = Path(__file__).resolve().parent
CARGO_FILE = str(_APP_DIR / "cargo_locations.json")
CARGO_DETAILS_FILE = str(_APP_DIR / "cargo_details.json")

def create_database():
    try:
        # postgres 기본 DB에 접속하여 새 DB 생성
        conn = psycopg2.connect(host="localhost", dbname="postgres", user="postgres", password="1234")
        conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)
        cur = conn.cursor()
        
        cur.execute("SELECT 1 FROM pg_catalog.pg_database WHERE datname = 'port_db'")
        exists = cur.fetchone()
        
        if not exists:
            cur.execute("CREATE DATABASE port_db")
            print("Database 'port_db' created successfully.")
        else:
            print("Database 'port_db' already exists.")
            
        cur.close()
        conn.close()
    except Exception as e:
        print(f"Error creating database: {e}")
        return False
    return True

def create_tables():
    try:
        conn = psycopg2.connect(host="localhost", dbname="port_db", user="postgres", password="1234")
        cur = conn.cursor()
        
        # relocation_tasks 테이블 생성
        cur.execute("""
            CREATE TABLE IF NOT EXISTS relocation_tasks (
                id SERIAL PRIMARY KEY,
                task_name VARCHAR(255) NOT NULL,
                status VARCHAR(50) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        cur.execute("DROP TABLE IF EXISTS cargos CASCADE")
        
        # cargos 테이블 생성
        cur.execute("""
            CREATE TABLE IF NOT EXISTS cargos (
                name VARCHAR(255) PRIMARY KEY,
                location VARCHAR(255) NOT NULL,
                container_id VARCHAR(255),
                cargo_type VARCHAR(255),
                note TEXT,
                base_aruco_id VARCHAR(255) DEFAULT '',
                floor INTEGER DEFAULT 1
            )
        """)
        
        conn.commit()
        print("Tables 'relocation_tasks' and 'cargos' created successfully.")
        
        # JSON 데이터 마이그레이션
        if os.path.exists(CARGO_FILE):
            with open(CARGO_FILE, 'r', encoding='utf-8') as f:
                registry = json.load(f)
        else:
            registry = {}
            
        if os.path.exists(CARGO_DETAILS_FILE):
            with open(CARGO_DETAILS_FILE, 'r', encoding='utf-8') as f:
                details = json.load(f)
        else:
            details = {}
            
        inserted = 0
        for name, location in registry.items():
            detail = details.get(name, {})
            container_id = detail.get("컨테이너ID", "")
            cargo_type = detail.get("화물종류", "")
            note = detail.get("비고", "")
            base_aruco_id = detail.get("기반ArUco", "")
            floor_str = detail.get("층수", "1")
            floor = int(floor_str) if floor_str.isdigit() else 1
            
            cur.execute("""
                INSERT INTO cargos (name, location, container_id, cargo_type, note, base_aruco_id, floor)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                ON CONFLICT (name) DO UPDATE SET
                location = EXCLUDED.location,
                container_id = EXCLUDED.container_id,
                cargo_type = EXCLUDED.cargo_type,
                note = EXCLUDED.note,
                base_aruco_id = EXCLUDED.base_aruco_id,
                floor = EXCLUDED.floor
            """, (name, location, container_id, cargo_type, note, base_aruco_id, floor))
            inserted += 1
            
        conn.commit()
        cur.close()
        conn.close()
        print(f"Migrated {inserted} cargos from JSON to PostgreSQL.")
        
    except Exception as e:
        print(f"Error creating tables / migrating data: {e}")

if __name__ == "__main__":
    if create_database():
        create_tables()
