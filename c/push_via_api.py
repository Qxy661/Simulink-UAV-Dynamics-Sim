"""把本地未推送的提交用 GitHub Git Data API 落到远端。

git push 在当前网络下对包含新对象的请求返回 403（小 POST 可以），
所以改用 REST：逐个 blob -> tree -> commit -> 更新 ref。结果是远端
多一个汇总提交（不是逐提交），消息里列出被压缩的本地提交。

用法：
    python c/push_via_api.py [--dry-run]
"""

import base64
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.request
import urllib.error

REPO = "Qxy661/Simulink-UAV-Dynamics-Sim"
BASE = os.environ.get("PUSH_BASE") or None  # 缺省取远端 main 当前提交


def sh(*args):
    r = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    return r.stdout.decode("utf-8", "replace").strip()


def token():
    return sh("gh", "auth", "token")


def api(method, path, payload=None, tok=None):
    url = "https://api.github.com" + path
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Authorization", "token " + (tok or token()))
    req.add_header("Accept", "application/vnd.github+json")
    req.add_header("Content-Type", "application/json")
    for attempt in range(5):
        try:
            with urllib.request.urlopen(req, timeout=60) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as e:
            body = e.read().decode()[:300]
            if e.code in (429, 500, 502, 503) and attempt < 4:
                time.sleep(2 * (attempt + 1))
                continue
            raise RuntimeError(f"{method} {path} -> {e.code}: {body}")
    raise RuntimeError("retries exhausted")


def local_files_at_head():
    out = sh("git", "ls-tree", "-r", "HEAD")
    files = {}
    for line in out.splitlines():
        meta, path = line.split("\t", 1)
        mode, typ, sha = meta.split()
        files[path] = (mode, sha)
    return files


def files_at_base():
    try:
        out = sh("git", "ls-tree", "-r", BASE)
    except subprocess.CalledProcessError:
        return {}
    files = {}
    for line in out.splitlines():
        meta, path = line.split("\t", 1)
        mode, typ, sha = meta.split()
        files[path] = (mode, sha)
    return files


def blob_bytes(sha):
    return subprocess.run(["git", "cat-file", "blob", sha],
                          capture_output=True, check=True).stdout


def main():
    dry = "--dry-run" in sys.argv
    head = sh("git", "rev-parse", "HEAD")
    head_tree = sh("git", "rev-parse", "HEAD^{tree}")
    print("HEAD", head[:7], "tree", head_tree[:7])

    global BASE
    if BASE is None:
        ref = api("GET", f"/repos/{REPO}/git/refs/heads/main")
        BASE = ref["object"]["sha"]
        print("base", BASE[:7])

    # BASE 是远端提交，本地可能还没有（远端由 API 汇总推送时历史不同）。
    # 没有就先 fetch 一次，后面的 ls-tree / git log 才有东西可比。
    try:
        sh("git", "cat-file", "-e", BASE)
    except subprocess.CalledProcessError:
        print("fetch base from origin ...")
        url = f"https://{token()}@github.com/{REPO}.git"
        sh("git", "-c", "credential.helper=", "-c", "http.proxy=",
           "-c", "http.sslVerify=false", "fetch", url, "main")
        try:
            sh("git", "cat-file", "-e", BASE)
        except subprocess.CalledProcessError:
            BASE = sh("git", "rev-parse", "FETCH_HEAD")
            print("base -> fetched", BASE[:7])

    if dry:
        diff = sh("git", "diff", "--name-status", BASE + "..HEAD")
        print(diff)
        return

    tok = token()

    # 直接复用本地已有的 tree 对象：GitHub 接受 POST /git/trees 携带
    # 已存在 blob 的 sha，但我们无法引用本地对象库，所以逐文件上传。
    now = local_files_at_head()
    base_files = files_at_base()

    changed = {p: v for p, v in now.items() if base_files.get(p) != v}
    removed = [p for p in base_files if p not in now]
    print(f"changed {len(changed)} removed {len(removed)}")

    entries = []
    cache_path = os.path.join(tempfile.gettempdir(), "uav_blob_cache.json")
    cache = {}
    if os.path.exists(cache_path):
        with open(cache_path, encoding="utf-8") as fh:
            cache = json.load(fh)
    for i, (path, (mode, sha)) in enumerate(sorted(changed.items()), 1):
        blob_sha = cache.get(sha)
        if blob_sha is None:
            data = blob_bytes(sha)
            blob = api("POST", f"/repos/{REPO}/git/blobs", {
                "content": base64.b64encode(data).decode(),
                "encoding": "base64",
            }, tok)
            blob_sha = blob["sha"]
            cache[sha] = blob_sha
            if i % 10 == 0:
                with open(cache_path, "w", encoding="utf-8") as fh:
                    json.dump(cache, fh)
                print(f"  blobs {i}/{len(changed)}")
        entries.append({
            "path": path,
            "mode": "100755" if mode == "100755" else "100644",
            "type": "blob",
            "sha": blob_sha,
        })

    with open(cache_path, "w", encoding="utf-8") as fh:
        json.dump(cache, fh)

    for path in removed:
        entries.append({"path": path, "mode": "100644", "type": "blob", "sha": None})

    base_commit = api("GET", f"/repos/{REPO}/git/commits/{BASE}", None, tok)
    tree = api("POST", f"/repos/{REPO}/git/trees", {
        "base_tree": base_commit["tree"]["sha"],
        "tree": entries,
    }, tok)

    log = sh("git", "log", "--oneline", BASE + "..HEAD")
    message = ("本地 15 个提交经 API 汇总推送（git push 在当前网络对含新对象的请求返回 403）\n\n"
               + log)
    commit = api("POST", f"/repos/{REPO}/git/commits", {
        "message": message,
        "tree": tree["sha"],
        "parents": [BASE],
    }, tok)
    print("new commit", commit["sha"][:7])

    api("PATCH", f"/repos/{REPO}/git/refs/heads/main", {"sha": commit["sha"]}, tok)
    print("ref updated ->", commit["sha"])


if __name__ == "__main__":
    main()
