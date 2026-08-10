import psycopg2

def reset_db():
    conn = psycopg2.connect(
        host="localhost",
        database="port_db",
        user="postgres",
        password="1234",
        port="5432"
    )
    cur = conn.cursor()
    
    # 기존 화물 전체 삭제 (초기화)
    cur.execute("DELETE FROM cargos")
    
    # 사용자 요청에 맞춘 신규 9개 화물 (C0 ~ C8, ArUco ID: 0~8)
    initial_cargos = [
        ("컨테이너_C0", "A-1-1", "0", "컨테이너", "초기 셋팅", "11", 1),
        ("컨테이너_C1", "A-1-2", "1", "컨테이너", "초기 셋팅", "12", 1),
        ("컨테이너_C2", "A-2-1", "2", "컨테이너", "초기 셋팅", "13", 1),
        ("컨테이너_C3", "A-2-2", "3", "팔레트", "초기 셋팅", "14", 1),
        ("컨테이너_C4", "A-3-1", "4", "팔레트", "초기 셋팅", "15", 1),
        ("컨테이너_C5", "A-3-2", "5", "팔레트", "초기 셋팅", "16", 1),
        ("컨테이너_C6", "항구", "6", "특수화물", "초기 셋팅", "", 1),
        ("컨테이너_C7", "항구", "7", "특수화물", "초기 셋팅", "", 1),
        ("컨테이너_C8", "항구", "8", "특수화물", "초기 셋팅", "", 1)
    ]
    
    for name, loc, container_id, ctype, note, base, floor in initial_cargos:
        cur.execute("""
            INSERT INTO cargos (name, location, container_id, cargo_type, note, base_aruco_id, floor)
            VALUES (%s, %s, %s, %s, %s, %s, %s)
        """, (name, loc, container_id, ctype, note, base, floor))
        
    conn.commit()
    cur.close()
    conn.close()
    print("DB의 화물이 9개(C0~C8)로 완전 초기화되었습니다!")

if __name__ == "__main__":
    reset_db()
