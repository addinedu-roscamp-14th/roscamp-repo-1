import psycopg2

def print_db_status():
    try:
        conn = psycopg2.connect(
            host="localhost",
            database="port_db",
            user="postgres",
            password="1234",
            port="5432"
        )
        cur = conn.cursor()
        
        # 새 스키마 확인
        cur.execute("SELECT name, location, container_id, base_aruco_id, floor FROM cargos ORDER BY name")
        rows = cur.fetchall()
        
        print("\n" + "="*70)
        print("🚚 [PostgreSQL DB 실시간 화물 목록 (cargos 테이블)]")
        print("="*70)
        print(f"{'화물명':<10} | {'위치':<12} | {'컨테이너 ID':<12} | {'Base ArUco':<12} | {'층수'}")
        print("-" * 70)
        
        for name, location, container_id, base_aruco_id, floor in rows:
            print(f"{name:<10} | {location:<12} | {container_id or '-':<12} | {base_aruco_id or '-':<12} | {floor}층")
            
        print("="*70)
        print(f"총 {len(rows)}개의 화물이 동기화되어 있습니다.\n")
        
        cur.close()
        conn.close()
    except Exception as e:
        print(f"\n❌ DB 접속 또는 조회 에러: {e}")
        print("혹시 python init_db.py 를 실행해서 스키마를 업데이트하셨나요?")

if __name__ == "__main__":
    print_db_status()
