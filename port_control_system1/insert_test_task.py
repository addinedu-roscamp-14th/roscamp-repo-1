import psycopg2
import time
import sys

def insert_test_task(task_name):
    try:
        conn = psycopg2.connect(
            host="localhost",
            database="port_db",
            user="postgres",
            password="1234",
            port="5432"
        )
        cur = conn.cursor()
        
        insert_query = "INSERT INTO cargos (name, location) VALUES (%s, %s)"
        cur.execute(insert_query, (task_name, '터미널 등록'))
        
        conn.commit()
        cur.close()
        conn.close()
        
        print(f"✅ DB에 새 화물 추가 완료: '{task_name}' (위치: 터미널 등록)")
        print("대시보드(혹은 화물 탭) 화면에 약 1초 안에 방금 추가한 화물이 나타나는지 확인해 보세요!")
        
    except Exception as e:
        print(f"❌ DB Insert 오류: {e}")

if __name__ == "__main__":
    # 터미널에서 인자를 주면 그 이름으로, 안 주면 기본 이름으로 생성
    new_task = sys.argv[1] if len(sys.argv) > 1 else f"외부 터미널 테스트 작업 - {int(time.time())}"
    insert_test_task(new_task)
