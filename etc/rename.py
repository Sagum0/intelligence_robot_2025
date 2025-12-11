import os

###############################
# 사용자 설정
###############################
TARGET_DIR = "/home/pc/intelrb_ws/etc/make_data/only_sqr"  # 파일들이 있는 폴더
NEW_PREFIX = "only_sqr_jo"                    # 새 파일 이름 앞부분
START_INDEX = 0                         # 시작 인덱스
###############################

# 폴더 내 파일 목록 (정렬)
file_list = sorted(os.listdir(TARGET_DIR))

index = START_INDEX

for filename in file_list:
    old_path = os.path.join(TARGET_DIR, filename)

    # 폴더 무시
    if os.path.isdir(old_path):
        continue

    # 파일명과 확장자 분리
    name, ext = os.path.splitext(filename)

    # 새로운 파일 이름 생성
    new_filename = f"{NEW_PREFIX}_{index}{ext}"
    new_path = os.path.join(TARGET_DIR, new_filename)

    # 파일명 변경
    os.rename(old_path, new_path)
    print(f"{filename} → {new_filename}")

    index += 1

print("파일명 변경 완료")
