

# Tips 
## VScodeを使ったRaspberry piへの接続
### VSCodeのインストール
以下手順にて実施する．
1. VSCodeのインストール
    - Windowsの場合，Windows Storeを開き`Visual studio code`で検索すると，該当ソフトが出てくるので，それをインストールする．
    ![Windows Store](images/windows_store.png)
1.  SSH用の拡張機能をインストールする．
    - VSCodeを実行し拡張機能（下図中の赤枠部分）をクリックする．
    ![拡張機能の選択](images/select_extention.png)
    - 検索バーで`SSH`と検索し，`Remote - SSH`を選択しインストールする．
    ![SSH拡張機能](images/search_ssh.png)
1. リモート機能の設定
    1. リモート（下図中の赤枠部分）をクリックする．
    ![remote機能の選択](images/select_remote.png)
    1. プルダウンメニューで`リモート(トンネル/SSH)`を選択する．
    ![プルダウンの選択](images/select_pulldown.png)
    1. SSHの文字の右側にある`+`のキーを押す．
    ![設定の追加](images/add_remotehost.png)
    1. コマンドとして`ssh irsl@xxx.xxx.xxx.xxx`と入れる．（`xxx.xxx.xxx.xxx`には自分の接続したいコンピュータのIPアドレスを入れる．） 
    ![設定の入力](images/set_remotehost.png)
    1. どの設定に加えるか出てくるが，ここでは一番上を選ぶ．
    ![設定ファイルの選択](images/select_setting_file.png)
    1. アップデートを行うと設定が追加される．
    ![設定の反映](images/update_setting.png)
### 接続方法
1. リモート（下図中の赤枠部分）をクリックする．
    ![remote機能の選択](images/select_remote.png)
1. プルダウンメニューで`リモート(トンネル/SSH)`を選択する．
    ![プルダウンの選択](images/select_pulldown.png)
1. 接続したいコンピュータのところの右側にある新しいウィンドウで接続をクリックする．
    ![リモートホストへ接続](images/open_newwindow.png)
### ターミナルの開き方
- 接続後`Ctrl + @`でターミナルが開く．
### ファイルアップロード
- 接続後に手元PCファイルをVSCodeの エクスプローラーへドラックアンドドロップすることでできる．(Jupyterからのファイルは直接アップロードできないので，一度手元のPCにダウンロードしてからアップロードすること．)
